
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
        ros::NodeHandle n;
    //ros::NodeHandle n1;
    caros::SerialDeviceSIProxy sd_sipA(n, "caros_universalrobot");
    //caros::SerialDeviceSIProxy sd_sipB(n1, "caros_universalrobot2");
    currentpos = deviceTree->getQ(state);

    Vector3D<> TA(req.tAx, req.tAy, req.tAz);
    bool goal = req.goal;
    RPY<> RA(req.rAz, req.rAy, req.rAx);
    Vector3D<> TB(req.tBx, req.tBy, req.tBz);
    RPY<> RB(req.rBz, req.rBy, req.rBx);
    ROS_INFO("Extracted Transform3D");
    Transform3D<> poseA(TA, RA.toRotation3D());
    Transform3D<> poseB(TB, RB.toRotation3D());

    double pruneepsi = 0.2;
    double SBLepsi = 0.2;

    rw::math::Transform3D<> goalPositionA = poseA;
    rw::math::Transform3D<> goalPositionB = poseB;
    vector<vector<Q>> goalposIntermidiate = findGoalConfig(workcell,deviceA, deviceB, state, poseA, poseB, switched);
    ROS_INFO("BLOB");
    goalposIntermidiate[1][0]=Q(6,3.201, -1.527, -2.34, -0.798, 1.558, 0.009);
    goalposIntermidiate[0][0]=Q(6,-0.044, -1.502, 2.334, -2.41, -1.574, -0.041);
    if (goalposIntermidiate[0].size()==0 || goalposIntermidiate[1].size()==0)
    {
      ROS_ERROR("No kinematic solution found!");
      res.goalReached=false;
    }
    ROS_INFO("Kinematic solution found!");
    ROS_INFO_STREAM("Sizes: " << goalposIntermidiate[0].size() <<goalposIntermidiate[1].size());

    std::cout << goalposIntermidiate[0][0] << endl << goalposIntermidiate[1][0] << endl;
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
        collision=false;
        if (!collision)
        {
          confCollisionFree = true;
          goalpos = createQ12(goalposIntermidiate[0], goalposIntermidiate[1], i, j);
          i = goalposIntermidiate[0].size();
          j = goalposIntermidiate[1].size();

        }
      }
    }
    if (!confCollisionFree) {
      ROS_ERROR("No combination of configurations found!");
      res.goalReached=false;
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
      ROS_INFO_STREAM("Current: " << currentpos);
      ROS_INFO_STREAM("goal:" << goalpos);
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
          Qb[j] = ainterpolated[i][j+6];
        }

        ROS_INFO("Moving Robot");
        sd_sipA.movePtp(Qa);
        //sd_sipB.movePtp(Qb);

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
      res.goalReached=false;
    }
    res.goalReached=true;
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
    ros::NodeHandle n2;
    caros::SerialDeviceSIProxy sd_sipA(n, "caros_universalrobot");
    //caros::SerialDeviceSIProxy sd_sipB(n2, "caros_universalrobot2");
    ROS_INFO_STREAM("SerialDeviceSIProxy created");
    workcell=caros::getWorkCell();
    deviceA =workcell->findDevice("UR10A");
    deviceB=workcell->findDevice("UR10B");
    // Get the state of the workcell
    ROS_INFO_STREAM("Getting state of workcell");
    state = workcell->getDefaultState();

    // Create a default collision detector
    CollisionDetector::Ptr detector = new CollisionDetector(workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    CollisionDetector::QueryResult data;

    Math::seed();

    ROS_INFO_STREAM("Making the vectorOfEnds");
    vector<kinematics::Frame*> vectorOfEnds;
    vectorOfEnds.push_back(deviceA->getEnd());
    vectorOfEnds.push_back(deviceB->getEnd());

    string refFrame = "RefFrame";
    string TopPlateName = "TopPlate";
    ROS_INFO_STREAM("making tree");
    deviceTree = new TreeDevice(workcell->findFrame(refFrame), vectorOfEnds, "TreeDevice", state);
    currentpos[0]=-0;currentpos[1]= -2.2689;currentpos[2]= 2.2689;currentpos[3]= -1.5708;currentpos[4]= -1.5708;currentpos[5]= 1.6689;
    currentpos[6]=3.1415;currentpos[7]= -0.8727;currentpos[8]= -2.2689;currentpos[9]= -1.5708;currentpos[10]= 1.5708;currentpos[11]= 0;
    deviceTree->setQ(currentpos,state);
    Q q1 = Q(6,-0, -2.2689, 2.2689, -1.5708, -1.5708, 1.6689);
    Q q2 = Q(6,3.1415, -0.8727, -2.2689, -1.5708, 1.5708,0);

    ros::Duration(0.1).sleep();  // In seconds

    sd_sipA.movePtp(q1);

    //sd_sipB.movePtp(q2);

    ROS_INFO_STREAM("Done");
    // -------------
    ros::ServiceServer serviceSBL = n.advertiseService("SBL_cmd", SBL_cmd);
    //---------------------------------------------------------------------------------------------------------
    ros::spin();



    return 0;
}
