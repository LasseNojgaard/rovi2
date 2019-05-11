
//------------------
// INCLUDES

// RobWork
#include"ros/ros.h"
//#include <rw/rw.hpp>
//#include <rw/math/Q.hpp>
//#include <rw/common.hpp>
#include <caros/common.h>
#include <caros/common_robwork.h>
#include <caros/caros_node_service_interface.h>
#include <caros/serial_device_si_proxy.h>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models.hpp>
#include <rw/common/Ptr.hpp>
#include <utility>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// SBL
#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/StateConstraint.hpp>

#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rw/models/TreeDevice.hpp>
#include <iostream>
#include <time.h>
#include "includes/inversekin.h"
#include "rovi2/SBL_cmd.h"
//---------
// Namespaces

// Robwork
using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::math;

using rw::math::Q;
using rw::kinematics::State;
using namespace rw::pathplanning;
using rw::proximity::CollisionDetector;
using rw::loaders::PathLoader;
using rwlibs::proximitystrategies::ProximityStrategyYaobi;
using namespace rwlibs::pathplanners;

using namespace rwlibs::proximitystrategies;

// SBL
using namespace rwlibs::pathplanners;

using namespace std;

//------------
// Global constants

clock_t timeStart;

Q Qa = Q(6);
Q Qb = Q(6);

double SBLepsi = 0.2;
double pruneepsi = 0.01;
int amountOfSteps = 100;
Q currentpos = Q(12);
Q goalpos = Q(12);

bool switched = false;

Q RobASquareRobBTri = Q(12);
Q RobATriRobBSquare = Q(12);

Q RobASquare(6, -0.8286, -1.1578,  1.8373, -2.2377, -1.5537, -0.8326);
Q RobATri(6, -0.5465, -1.3274,  2.0428, -2.2492, -1.6038, -0.5401);
Q RobBSquare(6,  3.9039, -2.0098, -1.7317, -0.9808,  1.6159,  3.9584);
Q RobBTri(6, 3.6644 , -1.8676, -1.8989, -1.0069, 1.6128, 3.5867);

rw::models::WorkCell::Ptr workcell;
rw::models::Device::Ptr deviceA;
rw::models::Device::Ptr deviceB;

TreeDevice::Ptr deviceTree;

caros::SerialDeviceSIProxy *sd_sipA;
caros::SerialDeviceSIProxy *sd_sipB;

State state;
//--------------------
// Functions

void line()
{
  cout << "----------------------------------------------------------" << endl;
}

rw::trajectory::Path<Q> SBL(Q inputConfiguration, Q outputConfiguration, CollisionDetector::Ptr aDetector, TreeDevice::Ptr aDeviceTree, State aState, double epsilon, WorkCell::Ptr aWorkcell)
{
  cout << "Beginning SBL" << endl;
  rw::trajectory::Path<Q> somePath;
  // Constraint
  rw::pathplanning::QConstraint::Ptr treeConstraint = rw::pathplanning::QConstraint::make(aDetector, aDeviceTree, aState);
  //rw::pathplanning::QConstraint::Ptr constraintB = rw::pathplanning::QConstraint::make(detector, deviceB, state);

  CollisionDetector collissiondect(aWorkcell, ProximityStrategyYaobi::make());
  //PKG , Check source fil for link fra yaobi.
  // Interpolate path includeret i robwork
  //CollisionDetector coldect(workcell, ProximityStrategyFactory::makeDefault());
  QConstraint::Ptr constraint = QConstraint::make(&collissiondect, aDeviceTree, aState);

  QEdgeConstraintIncremental::Ptr edgeconstraint = QEdgeConstraintIncremental::makeDefault(treeConstraint, aDeviceTree);

  QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(SBLSetup::make(treeConstraint, edgeconstraint, aDeviceTree, epsilon, epsilon));

  QSampler::Ptr cfreeQ = QSampler::makeConstrained(QSampler::makeUniform(aDeviceTree), treeConstraint, 100000);

  Q pos = aDeviceTree->getQ(aState);

  cout << "Making the trajectory from: " << endl;
  cout << pos << endl;

  cout << "To next configuration: " << endl;
  cout << outputConfiguration << endl;

  cout << "Reaching down the rabbithole" << endl;
  const bool ok = planner->query(inputConfiguration,outputConfiguration,somePath);
  cout << "Done checking nextQ" << endl;
  if(!ok)
  {
    std::cout << "Path " << "0" << " not found.\n" ;
  }
  else
  {
    cout << "Path found" << endl;
  }

  cout << "End SBL" << endl;

  return somePath;
}

bool extBinarySearch(double eps, Q qStart, Q qEnd, State& aState, CollisionDetector::Ptr aDetector, Device::Ptr aDevice) //true = no collision, false = collision
{
   Q deltaQ = qEnd-qStart;
    double n = deltaQ.norm2() / eps;
    int levels = ceil(log2(n));

    Q extDeltaQ = (deltaQ/deltaQ.norm2()) * pow(2, levels) * eps;

    Q currQ;
    Q step;
    int steps;

    CollisionDetector::QueryResult data;

    for(int i = 1; i <= levels; i++)
    {
        steps = pow(2, i-1);

        step = extDeltaQ / (double)steps;

        for(int j = 1; j <= steps; j++)
        {
            currQ = qStart + (j - 0.5)*step;

            aDevice->setQ(currQ, aState);
            if((currQ - qStart).norm2() <= deltaQ.norm2())
            {
                if(aDetector->inCollision(aState,&data))
                    return true;
            }
        }
    }
    return false;
}

rw::trajectory::Path<Q> pathPrune(rw::trajectory::Path<Q> aPath, double eps, State theState, CollisionDetector::Ptr theDetector, TreeDevice::Ptr theDevice)
{
  rw::trajectory::Path<Q> tempPath = aPath;

  size_t i = 0;
  while (i < tempPath.size()-2)
  {
    bool collision = extBinarySearch(eps, tempPath[i], tempPath[i+2], theState, theDetector, theDevice);
    //cout << "Object:" << i << " and " << i+2 << "collision: " << collision << endl;
    if (collision == 0)
    {
      tempPath.erase(tempPath.begin()+i+1);
      if (i>0)
      {
        i -= 1;
      }
    }
    else
    {
      i += 1;
    }
  }

  return tempPath;
}

rw::trajectory::Path<Q> interpolate(rw::trajectory::Path<Q> inputPath, int steps)
{
  cout << "Beginning interpolation" << endl;
  rw::trajectory::Path<Q> tempPath;

  //cout << "first" << endl;
  for (size_t i = 0; i < inputPath.size()-1; i++)
  {
    //cout << "second " << i << endl;
    double dist = abs(inputPath[i].norm2()-inputPath[i+1].norm2());
    //cout << "making amount; Distance: " << dist << endl;
    double amount = ceil(dist*steps);
    //cout << "making increment; Amount: " << amount << endl;
    Q increment = (inputPath[i+1]-inputPath[i]) / amount;
    //cout << "pushing back" << endl;
    tempPath.push_back(inputPath[i]);
    //cout << "from Q: " << inputPath[i] << endl << "to: " << inputPath[i+1] << endl;
    for (size_t j = 1; j < amount; j++)
    {
      //cout << "third " << j << endl;
      //cout << "Increment: " << increment*j << endl;
      tempPath.push_back(inputPath[i]+j*increment);
    }

  }
  //cout << "adding last" << endl;
  tempPath.push_back(inputPath[inputPath.size()-1]);
  cout << "Done interpolating" << endl;
  return tempPath;
}

bool SBL_cmd(rovi2::SBL_cmd::Request &req, rovi2::SBL_cmd::Response &res)
{
    ROS_INFO("In service");
    currentpos = deviceTree->getQ(state);

    Vector3D<> TA(req.tAx, req.tAy, req.tAz);
    bool goal = req.goal;
    RPY<> RA(req.rAz, req.rAy, req.rAx);
    Vector3D<> TB(req.tBx, req.tBy, req.tBz);
    RPY<> RB(req.rBz, req.rBy, req.rBx);
    ROS_INFO("Extracted Transform3D");
    Transform3D<> poseA(TA, RA.toRotation3D());
    Transform3D<> poseB(TB, RB.toRotation3D());

    double pruneepsi = 0.01;
    double SBLepsi = 0.01;

    rw::math::Transform3D<> goalPositionA = poseA;
    rw::math::Transform3D<> goalPositionB = poseB;
    vector<vector<Q>> goalposIntermidiate = findGoalConfig(deviceA, deviceB, state, poseA, poseB, switched);
    if (goalposIntermidiate[0].size()==0 || goalposIntermidiate[1].size()==0)
    {
      ROS_ERROR("No kinematic solution found!");
      return 1;
    }
    ROS_INFO("Kinematic solution found!");
    CollisionDetector::Ptr detector = new CollisionDetector(workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    bool confCollisionFree = false;
    bool collision = false;
    for (size_t i = 0; i < goalposIntermidiate[0].size(); i++)
    {
      deviceA->setQ(goalposIntermidiate[0][i], state);
      for (size_t j = 0; j < goalposIntermidiate[1].size(); j++)
      {
        deviceB->setQ(goalposIntermidiate[1][j], state);

        CollisionDetector::QueryResult data;
        collision = detector->inCollision(state,&data);

        if (!collision)
        {
          i = goalposIntermidiate[0].size();
          j = goalposIntermidiate[1].size();
          confCollisionFree = true;
          goalpos = createQ12(goalposIntermidiate[0], goalposIntermidiate[1], i, j);
        }
      }
    }
    if (!confCollisionFree) {
      ROS_ERROR("No combination of configurations found!");
      return 1;
    }
    ROS_INFO("Collision free combination found");
    rw::trajectory::Path<Q> agoodPath;
    if (goal==true)
    {
      if (switched==false)
      {
        agoodPath = SBL(currentpos, RobASquareRobBTri, detector, deviceTree, state, SBLepsi, workcell);
      }
      else
      {
        agoodPath = SBL(currentpos, RobATriRobBSquare, detector, deviceTree, state, SBLepsi, workcell);
      }
    }
    else
    {
      ROS_INFO("Starting SBL");
      agoodPath = SBL(currentpos, goalpos, detector, deviceTree, state, SBLepsi, workcell);
    }

    if (agoodPath.size()>0)
    {
      rw::trajectory::Path<Q> aprunedPath = pathPrune(agoodPath, pruneepsi, state, detector, deviceTree);
      rw::trajectory::Path<Q> ainterpolated = interpolate(aprunedPath,amountOfSteps);
      ROS_INFO("Pruned and iterpolated");
      Q Qa = Q(6);
      Q Qb = Q(6);
      for (size_t i = 0; i < ainterpolated.size(); i++)
      {
        for (size_t j = 0; j < 6; j++)
        {
          Qa[j] = ainterpolated[i][j];
        }
        for (size_t j = 6; j < 12; j++)
        {
          Qb[j] = ainterpolated[i][j];
        }
        ROS_INFO("Moving Robot");
        sd_sipA->movePtp(Qa);
        sd_sipB->movePtp(Qb);

      }
      ROS_INFO_STREAM("Qa: ");
      for (size_t i = 0; i < Qa.size(); i++)
      {
        ROS_INFO_STREAM(Qa[i] << ",");
      }
      ROS_INFO_STREAM("/n Qb: ");
      for (size_t i = 0; i < Qb.size(); i++)
      {
        ROS_INFO_STREAM(Qb[i] << ",");
      }

      ROS_INFO_STREAM("/n Goalposition: " << goalpos << endl);
      currentpos = goalpos;
    }
    else
    {
      ROS_ERROR("No path found !");
      return 1;
    }
    return true;
}





//----------
// Main
//---------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "SBL_cmd");
    ROS_INFO_STREAM("Node Started");
    for (size_t i = 0; i < 6; i++)
    {
      RobASquareRobBTri[i] = RobASquare[i];
      RobASquareRobBTri[i+6] = RobBTri[i];
    }
    for (size_t i = 0; i < 6; i++) {
      RobATriRobBSquare[i] = RobATri[i];
      RobATriRobBSquare[i+6] = RobBSquare[i];
    }
    string deviceAName = "UR10A";
    string deviceBName = "UR10B";

    ros::NodeHandle n;
    sd_sipA = new caros::SerialDeviceSIProxy(n, "caros_universalrobot");
    sd_sipB = new caros::SerialDeviceSIProxy(n, "caros_universalrobot2");
    ROS_INFO_STREAM("SerialDeviceSIProxy created");
    workcell=caros::getWorkCell();
    deviceA =workcell->findDevice("UR10A");
    deviceB=workcell->findDevice("UR10B");
    // Get the state of the workcell
    cout << "Getting state of workcell" << endl;
    state = workcell->getDefaultState();

    // Create a default collision detector
    CollisionDetector::Ptr detector = new CollisionDetector(workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    CollisionDetector::QueryResult data;

    // Create a configuration
    cout << "Collecting configuration data" << endl;

    Math::seed();

    cout << "Making the vectorOfEnds" << endl;
    vector<kinematics::Frame*> vectorOfEnds;
    vectorOfEnds.push_back(deviceA->getEnd());
    vectorOfEnds.push_back(deviceB->getEnd());

    string refFrame = "RefFrame";
    string TopPlateName = "TopPlate";
    cout << "making tree" << endl;
    deviceTree = new TreeDevice(workcell->findFrame(refFrame), vectorOfEnds, "TreeDevice", state);
    currentpos = deviceTree->getQ(state);
    cout << "Done" << endl;

    // -------------
    ros::ServiceServer serviceSBL = n.advertiseService("SBL_cmd", SBL_cmd);
    //---------------------------------------------------------------------------------------------------------
    ros::spin();



    return 0;
}

// TODO ----------------------

/*
    - Check the time needed to trim the paths and see how much time it cuts.
      Check and see wether it is recommended or not.
    - Make 10 paths and use the shortest of these to do the moves.
    - Check the interpolation
*/

    /*
    Q tempQA = Math::ranQ(qMinA,qMaxA);

    deviceA->setQ(qCollision, state);
    bool Ccollision = detector->inCollision(state, &data);
    cout << "Conf. in Ccollision: " << Ccollision << " should be in collision " <<  endl;
    cout << "The amount of colliding frames: " << data.collidingFrames.size() << endl;

    data.collidingFrames.clear();
    */

    /*
    //----Own implementation of SBL

    //--Collisionchecking
    //--Make new configurations
    Q qMinA = deviceA->getBounds().first;
    Q qMaxA = deviceA->getBounds().second;

    Q qMinB = deviceB->getBounds().first;
    Q qMaxB = deviceB->getBounds().second;

    bool collision = true;

    Q tempQA;
    Q tempQB;
    while(collision)
    {
      cout << "i" << endl;
        tempQA = Math::ranQ(qMinA, qMaxA);
        tempQB = Math::ranQ(qMinB, qMaxB);

        deviceA->setQ(tempQA,state);
        deviceB->setQ(tempQB,state);
        CollisionDetector::QueryResult data;
        collision = detector->inCollision(state,&data);

    }
    cout << "Collision free config: " << tempQA << " , "<< endl << tempQB << endl;
*/
