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
#include <iostream>

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
  //double roll,pitch,yaw;
  //world->getRobot(robotId)->getRotationRPY(roll,pitch,yaw);
  carOrientationAngleVector.push_back(0);
  steeringAngleVector.push_back(0);
  maxSteeringStep=1;
  minTurnRadius=0.1;
  trackSize=0.2;
  wheelBase=1;
  speed=1;
  timeStep=0.2;
  reverseGear=false;
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
  std::cerr<<_target[0]<<"  "<<_target[1]<<"  "<<_target[2]<<"\n";
  result = tryStepFromNodeWithHolonomicForConnect( _target, NNIdx );
  NNIdx = configVector.size() -1;
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
  return tryStepFromNodeWithHolonomic( _qtry, NNIdx );
}

/**
 * @function tryStepFromNode
 * @brief Tries to extend tree towards provided sample (must be overridden for MBP ) 
 */
RRT::StepResult RRT::tryStepFromNode( const Eigen::VectorXd &_qtry, int _NNIdx ) {
  std::cerr<<"kkk";
  std::cin>>_NNIdx;
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
  double qnearOrientation = -qnear[2];
  double qnearSteeringAngle = steeringAngleVector[_NNIdx];
  double newSteeringAngle;
  Eigen::VectorXd qtry = _qtry;
  qtry[2] = qnear[2];

  std::cerr<<"aa"<<qnear[0]<<"  "<<qnear[1]<<"  "<<-qnear[2]<<"\n";
  
  /// Compute direction and magnitude
  Eigen::VectorXd diff = qtry - qnear;
  Eigen::VectorXd sum = (qtry + qnear)/2;
  
  double dist = diff.norm();
  if( dist < stepSize )
  { 
    return STEP_REACHED; 
  }
  
  // perpendicular bisector between start and goal
  double a=sum[0],b=sum[1];
  std::cerr<<"a="<<a<<"b="<<b<<"\n";
  double slopePerpendicularBisector=(qtry[0]-qnear[0])/(qnear[1]-qtry[1]);
  std::cerr<<"slope="<<slopePerpendicularBisector<<"\n";
  double a1,b1,c1;
  double a2,b2,c2;
  
  a1 = slopePerpendicularBisector;
  b1 = -1;
  c1 = (a*a1)-b;
  
  a2 = 1;
  b2 = qnearOrientation;
  c2 = qnear[0] + (b2*qnear[1]);
  
  std::cerr<<a1<<"  "<<b1<<"  "<<c1<<"  "<<a2<<"  "<<b2<<"  "<<c2<<"\n";
  Eigen::VectorXd centerOfArc = qnear;
  centerOfArc[0] = ((c1*b2)-(c2*b1))/((a1*b2)-(a2*b1));
  centerOfArc[1] = ((a1*c2)-(a2*c1))/((a1*b2)-(a2*b1));
  double turnRadius = (centerOfArc-qnear).norm();
  std::cerr<<"center="<<centerOfArc[0]<<"  "<<centerOfArc[1]<<"  "<<turnRadius<<"\n";
  
    
  if( turnRadius < minTurnRadius )
  {
    return STEP_COLLISION;
  }
  std::cerr<<"dist="<<dist<<"\n";
  double angleSubtended = 2*asin(0.5*dist/turnRadius);
  double arcLength = angleSubtended*turnRadius;
  
  double radiusAngle = atan2(qnear[1]-centerOfArc[1],qnear[0]-centerOfArc[0]);
  
  
  
  // Scale this vector to stepSize and add to end of qnear
  Eigen::VectorXd qnew = centerOfArc;
  double deltaTheta = angleSubtended*(stepSize/arcLength);
  
  std::cerr<<angleSubtended<<"  "<<arcLength<<"  "<<deltaTheta<<"\n";
  //qnew[0] += diff*(stepSize/arcLength);
  double newRadiusAngle;
  std::cerr<<"angles"<<radiusAngle<<"  "<<qnearOrientation<<"\n";
  if( (radiusAngle< 0 && qnearOrientation<=PI/2 && qnearOrientation>-PI/2) ||
      (radiusAngle> 0 && (qnearOrientation>=PI/2 || qnearOrientation<-PI/2)) )
  {
    // right turn 
    newRadiusAngle = (radiusAngle+deltaTheta>PI)?(radiusAngle+deltaTheta-2*PI):(radiusAngle+deltaTheta);
    std::cerr<<"newradiusangle="<<newRadiusAngle<<"\n";
    qnew[0] += turnRadius*cos(newRadiusAngle);
    qnew[1] += turnRadius*sin(newRadiusAngle);
    newSteeringAngle = asin(wheelBase/(trackSize/2-turnRadius));
    //qnew[2] = 0;
    qnew[2] = -((newRadiusAngle+(PI/2)>PI)?(newRadiusAngle+(PI/2)-(2*PI)):(newRadiusAngle+(PI/2)));
    std::cerr<<"right";
  }
  else if( (radiusAngle< 0 && (qnearOrientation>PI/2 || qnearOrientation<=-PI/2)) ||
           (radiusAngle> 0 && qnearOrientation<PI/2 && qnearOrientation>=-PI/2) )
  {
    // left turn
    newRadiusAngle = (radiusAngle-deltaTheta<-PI)?(radiusAngle-deltaTheta+2*PI):(radiusAngle-deltaTheta);
    std::cerr<<"newradiusangle="<<newRadiusAngle<<"\n";
    qnew[0] += turnRadius*cos(newRadiusAngle);
    qnew[1] += turnRadius*sin(newRadiusAngle);
    newSteeringAngle = asin(wheelBase/(trackSize/2-turnRadius));
    qnew[2] = -((newRadiusAngle-(PI/2)<-PI)?(newRadiusAngle-(PI/2)+(2*PI)):(newRadiusAngle-(PI/2)));
    std::cerr<<"left";
  }
  else
  {
    return STEP_COLLISION;
  }
  std::cin>>a1;
  std::cerr<<"\n\n";
  //double roll, pitch, yaw;
  //world->getRobot(robotId)->getRotationRPY(roll,pitch,yaw);
  
  if( !checkCollisions(qnew) )
  {
    //std::cerr<<qnew[2]<<"  ";
    addNode( qnew, _NNIdx );
    //carOrientationAngleVector.push_back((newRadiusAngle-(PI/2)<-PI)?(newRadiusAngle-(PI/2)+(2*PI)):(newRadiusAngle-(PI/2)));
    steeringAngleVector.push_back(newSteeringAngle);
    //world->getRobot(robotId)->setRotationRPY(roll,pitch,newSteeringAngle);
    return STEP_PROGRESS;
  }
  else
  {
    return STEP_COLLISION;
  }
}

/**

 * @function tryStepFromNodeWithHolonomic
 * @brief Tries to extend tree towards provided sample (must be overridden for MBP ) 

 */
RRT::StepResult RRT::tryStepFromNodeWithHolonomicForConnect( const Eigen::VectorXd &_qtry, int _NNIdx ) {
  
  /// Calculates a new node to grow towards _qtry, check for collisions and adds to tree 
  Eigen::VectorXd qnear = configVector[_NNIdx];
  double qnearOrientation = -qnear[2];
  double qnearSteeringAngle = steeringAngleVector[_NNIdx];
  double newSteeringAngle;
  Eigen::VectorXd qtry = _qtry;
  qtry[2] = qnear[2];

  std::cerr<<"aa"<<qnear[0]<<"  "<<qnear[1]<<"  "<<-qnear[2]<<"\n";
  
  /// Compute direction and magnitude
  Eigen::VectorXd diff = qtry - qnear;
  Eigen::VectorXd sum = (qtry + qnear)/2;
  
  double dist = diff.norm();
  if( dist < stepSize )
  { 
    return STEP_REACHED; 
  }
  
  // perpendicular bisector between start and goal
  double a=sum[0],b=sum[1];
  std::cerr<<"a="<<a<<"b="<<b<<"\n";
  double slopePerpendicularBisector=(qtry[0]-qnear[0])/(qnear[1]-qtry[1]);
  std::cerr<<"slope="<<slopePerpendicularBisector<<"\n";
  double a1,b1,c1;
  double a2,b2,c2;
  
  a1 = slopePerpendicularBisector;
  b1 = -1;
  c1 = (a*a1)-b;
  
  a2 = 1;
  b2 = qnearOrientation;
  c2 = qnear[0] + (b2*qnear[1]);
  
  std::cerr<<a1<<"  "<<b1<<"  "<<c1<<"  "<<a2<<"  "<<b2<<"  "<<c2<<"\n";
  Eigen::VectorXd centerOfArc = qnear;
  centerOfArc[0] = ((c1*b2)-(c2*b1))/((a1*b2)-(a2*b1));
  centerOfArc[1] = ((a1*c2)-(a2*c1))/((a1*b2)-(a2*b1));
  double turnRadius = (centerOfArc-qnear).norm();
  std::cerr<<"center="<<centerOfArc[0]<<"  "<<centerOfArc[1]<<"  "<<turnRadius<<"\n";
  
    
  if( turnRadius < minTurnRadius )
  {
    return STEP_COLLISION;
  }
  std::cerr<<"dist="<<dist<<"\n";
  double angleSubtended = 2*asin(0.5*dist/turnRadius);
  double arcLength = angleSubtended*turnRadius;
  
  double radiusAngle = atan2(qnear[1]-centerOfArc[1],qnear[0]-centerOfArc[0]);
  
  
  
  // Scale this vector to stepSize and add to end of qnear
  Eigen::VectorXd qnew = centerOfArc;
  double deltaTheta = angleSubtended*(stepSize/arcLength);
  
  std::cerr<<angleSubtended<<"  "<<arcLength<<"  "<<deltaTheta<<"\n";
  //qnew[0] += diff*(stepSize/arcLength);
  double newRadiusAngle=radiusAngle;
  std::cerr<<"angles"<<radiusAngle<<"  "<<qnearOrientation<<"\n";
  RRT::StepResult result=STEP_PROGRESS;
  while( result == STEP_PROGRESS )
  {
    if( (radiusAngle< 0 && qnearOrientation<=PI/2 && qnearOrientation>-PI/2) ||
        (radiusAngle> 0 && (qnearOrientation>=PI/2 || qnearOrientation<-PI/2)) )
    {
      // right turn 
      newRadiusAngle = (newRadiusAngle+deltaTheta>PI)?(newRadiusAngle+deltaTheta-2*PI):(newRadiusAngle+deltaTheta);
      std::cerr<<"newradiusangle="<<newRadiusAngle<<"\n";
      qnew[0] = centerOfArc[0]+turnRadius*cos(newRadiusAngle);
      qnew[1] = centerOfArc[1]+turnRadius*sin(newRadiusAngle);
      newSteeringAngle = asin(wheelBase/(trackSize/2-turnRadius));
      //qnew[2] = 0;
      qnew[2] = -((newRadiusAngle+(PI/2)>PI)?(newRadiusAngle+(PI/2)-(2*PI)):(newRadiusAngle+(PI/2)));
      std::cerr<<"right";
    }
    else if( (radiusAngle< 0 && (qnearOrientation>PI/2 || qnearOrientation<=-PI/2)) ||
             (radiusAngle> 0 && qnearOrientation<PI/2 && qnearOrientation>=-PI/2) )
    {
      // left turn
      newRadiusAngle = (newRadiusAngle-deltaTheta<-PI)?(newRadiusAngle-deltaTheta+2*PI):(newRadiusAngle-deltaTheta);
      std::cerr<<"newradiusangle="<<newRadiusAngle<<"\n";
      qnew[0] = centerOfArc[0]+turnRadius*cos(newRadiusAngle);
      qnew[1] = centerOfArc[1]+turnRadius*sin(newRadiusAngle);
      newSteeringAngle = asin(wheelBase/(trackSize/2-turnRadius));
      qnew[2] = -((newRadiusAngle-(PI/2)<-PI)?(newRadiusAngle-(PI/2)+(2*PI)):(newRadiusAngle-(PI/2)));
      std::cerr<<"left";
    }
    else
    {
      return STEP_COLLISION;
    }
   // std::cin>>a1;
    std::cerr<<"\n\n";
    //double roll, pitch, yaw;
    //world->getRobot(robotId)->getRotationRPY(roll,pitch,yaw);
    
    if( !checkCollisions(qnew) )
    {
      //std::cerr<<qnew[2]<<"  ";
      addNode( qnew, _NNIdx );
      _NNIdx = configVector.size()-1;
      //carOrientationAngleVector.push_back((newRadiusAngle-(PI/2)<-PI)?(newRadiusAngle-(PI/2)+(2*PI)):(newRadiusAngle-(PI/2)));
      steeringAngleVector.push_back(newSteeringAngle);
      //world->getRobot(robotId)->setRotationRPY(roll,pitch,newSteeringAngle);
      if( getGap( _qtry, qnew) < stepSize )
        return STEP_REACHED;
    }
    else
    {
      return STEP_COLLISION;
    }
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
  //std::cerr<<configVector.size()<<std::endl;
  std::cerr<<"adding"<<_qnew[0]<<"  "<<_qnew[1]<<"  "<<_qnew[2]<<"\n";
  
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
double RRT::getGap( const Eigen::VectorXd &_target ){
    Eigen::VectorXd tempTarget = _target;
    tempTarget[2] = configVector[activeNode][2];
    return ( tempTarget - configVector[activeNode] ).norm();
}

/**
 * @function getGap

 * @brief Get the gap (Distance) between the closest node in the tree to the _target
 */
double RRT::getGap( const Eigen::VectorXd &_source , Eigen::VectorXd &_target ){
    _target[2] = _source[2];
    return ( _target - _source ).norm();
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
