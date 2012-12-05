/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 * @file RRT.cpp
 *
 * Authors:  Ana Huaman <ahuaman3@gatech.edu>, Tobias Kunz <tobias@gatech.edu>
 * Date: 10/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 */

#include "PathPlanner.h"

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner() {
  copyWorld = false;
  world = NULL;
}

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner( robotics::World &_world, 
                          bool _copyWorld, double _stepSize ) {
  
  copyWorld = _copyWorld;
  
  if( copyWorld ) {
    printf( "Do not use this option yet \n" );
  } else {
    world = &_world;
  }
  
  stepSize = _stepSize;
}

/**
 * @function ~PathPlanner
 * @brief Destructor
 */
PathPlanner::~PathPlanner() {
  
  if( copyWorld ) {
    delete world;
  }
}

/**
 * @function planPath
 * @brief Main function
 */
bool PathPlanner::planPath( int _robotId, 
			    const Eigen::VectorXi &_links, 
                            const Eigen::VectorXd &_start, 
                            const Eigen::VectorXd &_goal, 
                            bool _bidirectional, 
                            bool _connect, 
                            bool _greedy,
                            bool _smooth, 
                            unsigned int _maxNodes ) {


  //world->mRobots[_robotId]->setQuickDofs( _start ); // Other quick way
  world->getRobot(_robotId)->setDofs( _start, _links );
  if( world->checkCollision() )
    return false;
  
  world->getRobot(_robotId)->setDofs( _goal, _links );
  if( world->checkCollision() )
    return false;
  
  bool result;
  if( _bidirectional ) { 
    result = planBidirectionalRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes ); 
  } else {
    result = planSingleTreeRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes );
  }
  
  if( result && _smooth ) {
    smoothPath( _robotId, _links, path );
  }
  
  return result;
}


/**
 * @function planSingleRRT
 * @brief Finds a plan using a standard RRT
 */
bool PathPlanner::planSingleTreeRrt( int _robotId, 
                                     const Eigen::VectorXi &_links, 
                                     const Eigen::VectorXd &_start, 
                                     const Eigen::VectorXd &_goal, 
                                     bool _connect, 
                                     bool _greedy,
                                     unsigned int _maxNodes ) {
  
  RRT rrt( world, _robotId, _links, _start, stepSize );
  RRT::StepResult result = RRT::STEP_PROGRESS;
  
  double smallestGap = DBL_MAX;
  const double greedyProbabilityThreshold = 0.50;
  while ( result != RRT::STEP_REACHED && smallestGap > stepSize )
  {
    if( _greedy )/** greedy section */
    {
      double randValue = (double)rand() / ((double)RAND_MAX);
      if( _connect )/** greedy and connect */
      {
        randValue>greedyProbabilityThreshold?rrt.connect(_goal):rrt.connect();
    	}
      else /** greedy and NO connect */
      {
        randValue>greedyProbabilityThreshold?rrt.tryStep(_goal):rrt.tryStep();
      }
    }
    else/** NO greedy section */
    
    {
      if( _connect )/** NO greedy and Connect */
      {
      	rrt.connect();
	    	
      }
      else/** No greedy and No connect -- PLAIN RRT */
      {
      	rrt.tryStep();
      }
    }
    if( _maxNodes > 0 && rrt.getSize() > _maxNodes ) {
      printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
      return false;
    }
    
    double gap = rrt.getGap( _goal );
    if( gap < smallestGap ) {
      smallestGap = gap;
      std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " << rrt.configVector.size() << std::endl;
    }
  } // End of while
  
    /// Save path  
  printf(" --> Reached goal! : Gap: %.3f \n", rrt.getGap( _goal ) );
  rrt.tracePath( rrt.activeNode, path, false );
  
  return true;
}

/**
 * @function planBidirectionalRRT
 * @brief Grows 2 RRT (Start and Goal)
 */
bool PathPlanner::planBidirectionalRrt( int _robotId, 
                                        const Eigen::VectorXi &_links, 
                                        const Eigen::VectorXd &_start, 
                                        const Eigen::VectorXd &_goal, 
                                        bool _connect,
                                        bool _greedy, // no effect here
                                        unsigned int _maxNodes ) {
  
  // ============= YOUR CODE HERE ======================
  // HINT: Remember trees grow towards each other!
  RRT rrtFromStart( world, _robotId, _links, _start, stepSize );
  RRT rrtFromGoal( world, _robotId, _links, _goal, stepSize );
  RRT *rrtA, *rrtB;
  RRT::StepResult result = RRT::STEP_PROGRESS;
  if( _greedy )
  {
    std::cout << "Warning: Option \"Greedy\" does not apply to bidirectional case\n";
  }
  double smallestGap = DBL_MAX, gap;
  const double greedyProbabilityThreshold = 0.50;
  bool expandFromStart = true;
  Eigen::VectorXd qtry, *nodeA;
  while ( result != RRT::STEP_REACHED && smallestGap > stepSize )
  {
    double randValue = (double)rand() / ((double)RAND_MAX);
    expandFromStart = !expandFromStart;
    if(expandFromStart)
    {
      rrtA = &rrtFromStart; nodeA = (Eigen::VectorXd*)&_start;
      rrtB = &rrtFromGoal;
    }
    else
    {
      rrtA = &rrtFromGoal; nodeA = (Eigen::VectorXd*)&_goal;
      rrtB = &rrtFromStart;
    }
    if( _connect )/** greedy and connect */
    {
      if(randValue > greedyProbabilityThreshold)
      {
        int nearestToStart = rrtB->getNearestNeighbor(*nodeA);
        rrtA->connect(rrtB->configVector[nearestToStart]);
        gap = rrtA->getGap(rrtB->configVector[nearestToStart]);
      }
      else
      {
        qtry = rrtA->getRandomConfig();
        if(rrtA->connect(qtry))
        {
          rrtB->connect(qtry);
        }
        gap = rrtB->getGap( rrtA->configVector[rrtA->getNearestNeighbor(qtry)] );
      }
    }
    
    else /** greedy and NO connect */
    {
      if(randValue > greedyProbabilityThreshold)
      {
        int nearestToStart = rrtB->getNearestNeighbor(*nodeA);
        rrtA->tryStep(rrtB->configVector[nearestToStart]);
        gap = rrtA->getGap(rrtB->configVector[nearestToStart]);
      }
      else
      {
        qtry = rrtA->getRandomConfig();
        if(rrtA->tryStep(qtry))
        {
          rrtB->tryStep(qtry);
        }
        gap = rrtB->getGap( rrtA->configVector[rrtA->getNearestNeighbor(qtry)] );
      }
    }
    if( _maxNodes > 0 && rrtFromStart.getSize() + rrtFromGoal.getSize() > _maxNodes ) {
      printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
      return false;
    }
    
    //gap = 1;//rrtFromGoal.getGap( rrtFromStart.configVector[rrtFromStart.getNearestNeighbor(qtry)] );
    if( gap < smallestGap )
    {
      smallestGap = gap;
      std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " 
                << rrtFromStart.configVector.size() << " and "
                << rrtFromGoal.configVector.size() << std::endl;
    }
  } // End of while
  
    /// Save path  
  printf(" --> Reached goal! : Gap: %.3f \n", smallestGap );
  rrtFromStart.tracePath( rrtFromStart.activeNode, path, false );
  rrtFromGoal.tracePath( rrtFromGoal.activeNode, path, true );
  
  return true;
  // ===================================================	
  
}


/**
 * @function checkPathSegment
 * @brief True iff collision-free
 */
bool PathPlanner::checkPathSegment( int _robotId, 
                                    const Eigen::VectorXi &_links, 
                                    const Eigen::VectorXd &_config1, 
                                    const Eigen::VectorXd &_config2 ) const {
  
  int n = (int)((_config2 - _config1).norm() / stepSize );
  //std::cout<<n<<"  "<<stepSize<<std::endl;
  for( int i = 0; i < n; i++ ) {
    Eigen::VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
    world->getRobot(_robotId)->setDofs( conf, _links );
    world->getRobot(_robotId)->update();
    if( world->checkCollision() ) {
      return false;
    }
  }
  
  return true;
}

// Counts the length of an existing path
unsigned int countDist(std::list<Eigen::VectorXd>::iterator a, std::list<Eigen::VectorXd>::iterator b) {
	unsigned int length = 0;
	std::list<Eigen::VectorXd>::iterator cur = a;
	while (cur != b) {
		++length;
		++cur;
	}
	return length;
}

// Creates a path between 2 nodes
std::list<Eigen::VectorXd> PathPlanner::createPath(int _robotId, 
                                      const Eigen::VectorXi &_links, 
                                      const Eigen::VectorXd &_config1, 
                                      const Eigen::VectorXd &_config2)
{
  std::list<Eigen::VectorXd> path;
  int n = (int)((_config2 - _config1).norm() / stepSize );
//  std::cout<<n<<std::endl;
  for( int i = 0; i < n; i++ )
  {
    Eigen::VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
    world->getRobot(_robotId)->setDofs( conf, _links );
    world->getRobot(_robotId)->update();
    if( world->checkCollision() ) {
      return std::list<Eigen::VectorXd>();
    }
    path.push_back(conf);
  }
  
  return path;
}


/**
 * @function smoothPath
 */
void PathPlanner::smoothPath( int _robotId, 
                              const Eigen::VectorXi &_links, 
                              std::list<Eigen::VectorXd> &_path ) {
  
  // =========== YOUR CODE HERE ==================
  // HINT: Use whatever technique you like better, first try to shorten a path and then you can try to make it smoother
  //int i,n;
  //int node1=(int)rand()%n, node2=(int)rand()%n, tempNode;
  /*std::list<Eigen::VectorXd>::iterator it1=_path.begin(),it2=_path.begin();
  it2++; it2++;
  while(it1!=_path.end() && it2!=_path.end())
  {*/
    /*n = _path.size();
    node1=(int)rand()%n;
    node2=(int)rand()%n;
    if( node1 == node2)
    {
      continue;
    }
    else if(node1 > node2)
    {
      tempNode = node1;
      node1 = node2;
      node2 = node1;
    }
    it1 = _path.begin();
    it2 = it1;
    it1 += node1;
    it2 += node2;*//*
    if(checkPathSegment(_robotId, _links, *it1, *it2))
    {
      _path.erase(it1,it2);
      it1 = it2;
      it2++;
      it2++;
      //_links.erase(_links.begin()+node1,_links.begin()+node2);
      //_links.insert(_links.begin()+node1,)
    }
    
    ++it1;
    ++it2;
  }
*/



	std::cout << "Starting path shortening with simple search..." << _links.size() << std::endl;
	std::cout << "start path length: " << _path.size() << std::endl;

	std::list<Eigen::VectorXd>::iterator start_point=_path.begin(),end_point=_path.end();
	end_point--;

	// loop while start has not reached the end of the path
	while (start_point != _path.end())
	{
		if (start_point == end_point)
		{
		//std::cout << "End iteration\n";
			++start_point;
			end_point = _path.end();
			end_point--;
		}
		else
		{
		  //Eigen::VectorXd start_node=*start_point, end_node=*end_point;
			std::list<Eigen::VectorXd> segment = createPath(_robotId,_links,*start_point, *end_point);
			double curDist =  countDist(start_point, end_point) * stepSize;
			double shortcutDist = segment.size() * stepSize;
			if (segment.size()>0 && shortcutDist < curDist)
			{
			  std::cout << "Shortcut length: " << shortcutDist << std::endl;
				std::cout << "Current distance: " << curDist << std::endl;
				std::cout << "Found shortcut!" << std::endl;
				// reconstruct path
				// first segment
				std::list<Eigen::VectorXd> new_path(_path.begin(), start_point);
				// middle segment
				new_path.insert(new_path.end(), segment.begin(), segment.end());
				// last segment
				new_path.insert(new_path.end(), end_point, _path.end());

        std::cout << "New path length: " << new_path.size() << std::endl;
				
				// replace optimized
				_path = new_path;

				start_point = _path.begin();
				end_point = _path.end();
				end_point--;
			}
			else
			{
			  --end_point;
			}
		}
	}
	std::cout << "Finished Optimizing!  Final path length: " << _path.size() << std::endl;  
  return;
  // ========================================	
}


