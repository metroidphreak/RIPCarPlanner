/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 * @file RRT.cpp
 *
 * Authors: Tobias Kunz <tobias@gatech.edu>, Ana Huaman <ahuaman3@gatech.edu>
 * Date: 10/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 */

#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "RRT.h"

#define PI 3.14159265

/**
 * @function RRT
 * @brief Constructor
 */
RRT::RRT( robotics::World *_world, 
          int _robotId, 
          const Eigen::VectorXi &_links, 
          const Eigen::VectorXd &_root, 
          double _stepSize ) {
  
  /// Initialize some member variables
  world = _world; 
  robotId = _robotId;
  links = _links; 
  ndim = links.size();
  stepSize = _stepSize;
  
  /// Initialize random generator   
  srand( time(NULL) );
  
  /// Create kdtree and add the first node (start) 
  kdTree = kd_create( ndim );
  addNode( _root, -1 ); 
}

/**
 * @function ~RRT()
 * @brief Destructor
 */
RRT::~RRT() {
    kd_free( kdTree );
}

/**
 * @function connect
 * @brief Connect the closest node with random target, stop until it is reached or until it collides
 */
bool RRT::connect() {
  Eigen::VectorXd qtry = getRandomConfig();
  return connect( qtry );
}

/**
 * @function connect
 * @brief Connect the closest node with _target, stop until it is reached or until it collides
 */
bool RRT::connect( const Eigen::VectorXd &_target ) {
  
  int NNIdx = getNearestNeighbor( _target );
  StepResult result = STEP_PROGRESS;
  int i = 0;
  while( result == STEP_PROGRESS ) {
    
    result = tryStepFromNode( _target, NNIdx );
    NNIdx = configVector.size() -1;
    i++; 
  }
  return ( result == STEP_REACHED );
  
}

/**
 * @function tryStep
 * @brief Try to advance one stepSize towards a random target
 */
RRT::StepResult RRT::tryStep() {
    Eigen::VectorXd qtry = getRandomConfig();
    return tryStep( qtry );
}

/**
 * @function tryStep
 * @brief Try to advance one stepSize towards _qtry
 */
RRT::StepResult RRT::tryStep( const Eigen::VectorXd &_qtry ) {
  int NNIdx = getNearestNeighbor( _qtry );
  return tryStepFromNode( _qtry, NNIdx );
}

/**
 * @function tryStepFromNode
 * @brief Tries to extend tree towards provided sample (must be overridden for MBP ) 
 */
RRT::StepResult RRT::tryStepFromNode( const Eigen::VectorXd &_qtry, int _NNIdx ) {
  
  /// Calculates a new node to grow towards _qtry, check for collisions and adds to tree 
  Eigen::VectorXd qnear = configVector[_NNIdx];
  
  /// Compute direction and magnitude
  Eigen::VectorXd diff = _qtry - qnear;
  double dist = diff.norm();
  
  if( dist < stepSize ) { 
    return STEP_REACHED; 
  }
  
  /// Scale this vector to stepSize and add to end of qnear
  Eigen::VectorXd qnew = qnear + diff*(stepSize/dist);
  
  if( !checkCollisions(qnew) ) {
    addNode( qnew, _NNIdx );
    return STEP_PROGRESS;
  } else {
    return STEP_COLLISION;
  }
  
}

/**
 * @function tryStepFromNodeWithHolonomic
 * @brief Tries to extend tree towards provided sample (must be overridden for MBP ) 

 */
RRT::StepResult RRT::tryStepFromNodeWithHolonomic( const Eigen::VectorXd &_qtry, int _NNIdx ) {
  
  /// Calculates a new node to grow towards _qtry, check for collisions and adds to tree 
  Eigen::VectorXd qnear = configVector[_NNIdx];
  double qnearOrientation = carOrientationAngleVector[_NNIdx];
  double qnearSteeringAngle = steeringAngleVector[_NNIdx];
  double newSteeringAngle;
  
  /// Compute direction and magnitude
  Eigen::VectorXd diff = _qtry - qnear;
  Eigen::VectorXd sum = (_qtry + qnear)/2;
  
  double dist = diff.norm();
  if( dist < stepSize )
  { 
    return STEP_REACHED; 
  }
  
  // perpendicular bisector between start and goal
  double a=sum[0]/2,b=sum[1]/2;
  double slopePerpendicularBisector=(_qtry[0]-qnear[0])/(qnear[1]-_qtry[1]);
  
  double a1,b1,c1;
  double a2,b2,c2;
  
  a1 = slopePerpendicularBisector;
  b1 = -1;
  c1 = (a*a1)-b;
  
  a2 = 1;
  b2 = tan(qnearSteeringAngle);
  c2 = qnear[0] + (b2*qnear[1]);
  
  Eigen::VectorXd centerOfArc = qnear;
  centerOfArc[0] = ((c1*b2)-(c2*b1))/((a1*b2)-(a2*b1));
  centerOfArc[1] = ((a1*c2)-(a2*c1))/((a1*b2)-(a2*b1));
  
  double turnRadius = (centerOfArc-qnear).norm();
  
  if( turnRadius < minTurnRadius )
  {
    return STEP_COLLISION;
  }
  
  double angleSubtended = 2*atan(0.5*dist/turnRadius);
  double arcLength = angleSubtended*turnRadius;
  
  double radiusAngle = atan2(qnear[1]-centerOfArc[1],qnear[0]-centerOfArc[0]);
  
  // Scale this vector to stepSize and add to end of qnear
  Eigen::VectorXd qnew = centerOfArc;
  double deltaTheta = angleSubtended*(stepSize/arcLength);
  //qnew[0] += diff*(stepSize/arcLength);
  double newRadiusAngle;
  if( (radiusAngle> 0 && qnearOrientation<=PI/2 && qnearOrientation>-PI/2) ||
      (radiusAngle< 0 && (qnearOrientation>=PI/2 || qnearOrientation<-PI/2)) )
  {
    // right turn 
    newRadiusAngle = (radiusAngle-deltaTheta<-PI)?(radiusAngle-deltaTheta+2*PI):(radiusAngle-deltaTheta);
    qnew[0] += turnRadius*cos(newRadiusAngle);
    qnew[1] += turnRadius*sin(newRadiusAngle);
    newSteeringAngle = asin(wheelBase/(trackSize/2-turnRadius));
  }
  else if( (radiusAngle> 0 && (qnearOrientation>PI/2 || qnearOrientation<=-PI/2)) ||
           (radiusAngle< 0 && qnearOrientation<PI/2 && qnearOrientation>=-PI/2) )
  {
    // left turn
    newRadiusAngle = (radiusAngle+deltaTheta>PI)?(radiusAngle+deltaTheta-2*PI):(radiusAngle+deltaTheta);
    qnew[0] += turnRadius*cos(newRadiusAngle);
    qnew[1] += turnRadius*sin(newRadiusAngle);
    newSteeringAngle = asin(wheelBase/(trackSize/2-turnRadius));
  }
  else
  {
    return STEP_COLLISION;
  }
  double roll, pitch, yaw;
  world->getRobot(robotId)->getRotationRPY(roll,pitch,yaw);
  
  if( !checkCollisions(qnew) )
  {
    addNode( qnew, _NNIdx );
    carOrientationAngleVector.push_back((newRadiusAngle-(PI/2)<-PI)?(newRadiusAngle-(PI/2)+(2*PI)):(newRadiusAngle-(PI/2)));
    steeringAngleVector.push_back(newSteeringAngle);
    world->getRobot(robotId)->setRotationRPY(roll,pitch,newSteeringAngle);
    return STEP_PROGRESS;
  } else {
    return STEP_COLLISION;
  }
  
}


/**
 * @function addNode
 * @brief Add _qnew to tree with parent _parentId
 * @return id of node added
 */
int RRT::addNode( const Eigen::VectorXd &_qnew, int _parentId )
{
  /// Update graph vectors -- what does this mean?
  configVector.push_back( _qnew );
  parentVector.push_back( _parentId );
  
  uintptr_t id = configVector.size() - 1;
  kd_insert( kdTree, _qnew.data(), (void*) id ); //&idVector[id];  /// WTH? -- ahq

  activeNode = id;
  return id;
}  

/**
 * @function getRandomConfig
 * @brief Samples a random point for qtmp in the configuration space,
 * bounded by the provided configuration vectors (and returns ref to it)
 */
Eigen::VectorXd RRT::getRandomConfig() {
  Eigen::VectorXd config( ndim );
  for( unsigned int i = 0; i < ndim; i++ ) {
    double minVal = world->getRobot(robotId)->getDof(links[i])->getMin();
    double maxVal = world->getRobot(robotId)->getDof(links[i])->getMax();
    config[i] = randomInRange( minVal, maxVal ); 
  }
  
  return config;       
}

/**
 * @function getNearestNeighbor
 * @brief Returns Nearest Neighbor index to query point
 */
int RRT::getNearestNeighbor( const Eigen::VectorXd &_qsamp ) {

    struct kdres* result = kd_nearest( kdTree, _qsamp.data() );
    uintptr_t nearest = (uintptr_t)kd_res_item_data(result);

    activeNode = nearest;
    return nearest;
   
}

/**
 * @function getGap
 * @brief Get the gap (Distance) between the closest node in the tree to the _target
 */
double RRT::getGap( const Eigen::VectorXd &_target ) {
    return ( _target - configVector[activeNode] ).norm();
}

/**
 * @function tracePath
 * @brief Traces the path from some node to the initConfig node
 */
void RRT::tracePath( int _node, 
                     std::list<Eigen::VectorXd> &_path, 
                     bool _reverse ) {
  
    int x = _node;
    
    while( x != -1 ) {
      if( !_reverse ) {
	_path.push_front( configVector[x] );
      } else {
	_path.push_back( configVector[x] );
      }
      x = parentVector[x];
    }
}

/**
 * @function checkCollisions
 * @brief Returns true if collisions. If it is false, we are cool.
 */
bool RRT::checkCollisions( const Eigen::VectorXd &_config ) {
  world->getRobot(robotId)->setDofs( _config, links );
  world->getRobot(robotId)->update();
  return world->checkCollision();
}

/**
 * @function getSize
 * @brief Returns size of the tree
 */
unsigned int RRT::getSize() {
    return configVector.size();
}

/**
 * @function randomInRange
 * @brief Get random number between min and max
 */
double RRT::randomInRange( double _min, double _max ) {

    return _min + ((_max - _min) * ((double)rand() / ((double)RAND_MAX + 1))); 
}
