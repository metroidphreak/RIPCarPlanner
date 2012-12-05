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
  
  while ( result != RRT::STEP_REACHED && smallestGap > stepSize ) {
    
    /** greedy section */
    if( _greedy ) {

      /** greedy and connect */
      if( _connect ) {
        if(rand()%2 == 0)
	      {
	        rrt.connect();
	      } else
	      {
	        rrt.connect(_goal);
	      }
	
	// ================ YOUR CODE HERE ===============
	
	// ===============================================
	      
	/** greedy and NO connect */
      } else {
	
	// ================== YOUR CODE HERE ===================
	
	// =====================================================

	      //std::cout << rrt.getSize() << std::endl;
	      if(rand()%2 == 0)
	      {
	        result = rrt.tryStep();
	      } else
	      {
	        result = rrt.tryStep(_goal);
	      }
      }
      
      /** NO greedy section */
    } else {
      
      /** NO greedy and Connect */
      if( _connect ) {
	      rrt.connect();
	
	/** No greedy and No connect -- PLAIN RRT */
      } else {
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
  printf(" --> Reached goal! : Gap: %.3f ", rrt.getGap( _goal ) );
  rrt.tracePath( rrt.activeNode, path, false );
  printf(" --> Returning \n");
  
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
  
  RRT rrtF( world, _robotId, _links, _start, stepSize );
  RRT rrtB( world, _robotId, _links, _goal, stepSize );

  RRT::StepResult result = RRT::STEP_PROGRESS;
  
  double smallestGap = DBL_MAX;

  RRT *currentRRT;
  RRT *otherRRT;
  Eigen::VectorXd target;

  if(_greedy)
  {
    std::cout << "Greedy has no effect in bidirectional" << std::endl;
  }
  
  while ( result != RRT::STEP_REACHED && smallestGap > stepSize ) {

    //set rrt being expanded to the one with the fewest nodes
    if(rrtF.getSize() > rrtB.getSize())//currentRRT == &rrtF)
	  {
	    currentRRT = &rrtB;
	    otherRRT = &rrtF;
	  } else
	  {
	    currentRRT = &rrtF;
	    otherRRT = &rrtB;
	  }
	      //compute node on other tree to expand toward
    target = findClosestNode(currentRRT, otherRRT);
	  
    if( _connect ) {
	
	// ================ YOUR CODE HERE ===============
      if(rand()%5 == 0)
	    {
        currentRRT->connect();
      } else
      {
        currentRRT->connect(target);
      }
	// ===============================================
	      
	/** NO connect */
    } else {
	
	// ================== YOUR CODE HERE ===================
	
	// =====================================================
      if(rand()%5 == 0)
      {
        currentRRT->tryStep();
      } else
      {
        currentRRT->tryStep(target);
      }
    }

    target = findClosestNode(currentRRT, otherRRT);

    if( _maxNodes > 0 && rrtF.getSize() + rrtB.getSize() > _maxNodes ) {
      printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
      return false;
    }

    
    double gap = currentRRT->getGap( target );
    if( gap < smallestGap ) {
      smallestGap = gap;
      std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size forward: " << rrtF.configVector.size() << " backward: " << rrtB.configVector.size() << std::endl;
    }
    	  
  } // End of while

  std::cout << rrtF.activeNode << " " << rrtB.activeNode << std::endl;

  int acNode = rrtF.activeNode;
  rrtF.activeNode = rrtF.getNearestNeighbor(findClosestNode(&rrtB, &rrtF));
  if(acNode == rrtF.activeNode)
  {
    rrtB.activeNode = rrtB.getNearestNeighbor(findClosestNode(&rrtF, &rrtB));
  }

  std::cout << rrtF.activeNode << " " << rrtB.activeNode << std::endl;
  
  printf(" --> Reached goal! : Gap: %.3f \n", currentRRT->getGap( target ) );
  rrtF.tracePath( rrtF.activeNode, path, false );  
  rrtB.tracePath( rrtB.activeNode, path, true );

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
  
  for( int i = 0; i < n; i++ ) {
    Eigen::VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
    world->getRobot(_robotId)->setDofs( conf, _links );
    if( world->checkCollision() ) {
      return false;
    }
  }
  
  return true;
}

/**
 * @function smoothPath
 */
void PathPlanner::smoothPath( int _robotId, 
                              const Eigen::VectorXi &_links, 
                              std::list<Eigen::VectorXd> &_path ) {
  
  // =========== YOUR CODE HERE ==================
  // HINT: Use whatever technique you like better, first try to shorten a path and then you can try to make it smoother
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
  return;
  // ========================================	
}

// Counts the length of an existing path
unsigned int PathPlanner::countDist(std::list<Eigen::VectorXd>::iterator a, std::list<Eigen::VectorXd>::iterator b) {
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

Eigen::VectorXd PathPlanner::findClosestNode(RRT *currentRRT, RRT *otherRRT)
{
  std::vector<Eigen::VectorXd>::const_iterator vecIt;
  Eigen::VectorXd closest;
  double currGap = 10000.0;
  for(vecIt = otherRRT->configVector.begin(); vecIt != otherRRT->configVector.end(); vecIt++)
  {
    if(currentRRT->getGap(*vecIt) < currGap)
    {
      currGap = currentRRT->getGap(*vecIt);
      closest = *vecIt;
    }
  }
  return closest;
}

