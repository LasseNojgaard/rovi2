#include "ros/ros.h"
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/kinematics.hpp>
#include <rw/kinematics/JacobianIKSolver.hpp>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;

Device::Ptr device1;
Device::Ptr device2;
State state;
Transform3D<> relatePosetoBase(WorkCell::Ptr wc, Device::Ptr device ,Transform3D<> pose, State state){
  Frame* table=wc->findFrame("TopPlate");
  Frame* base=device->getBase();
  Transform3D<> table2base= table->fTf(base,state);
  return table*table2base;
}

rw::math::Q findGoalConfig(Device::Ptr device_1,Device::Ptr device_2, State state, Transform3D<> pose1, Transform3D pose2) {
  //create solvers
  Q result(12);
  rw::invkin::JacobianIKSolver solver1(device_1,state);
  rw::invkin::JacobianIKSolver solver2(device_2,state);

  //solve
  Q inter1=solver1.solve(pose1,state);
  Q inter2=solver2.solve(pose2,state);

  for(int i=0;i<result.size()/2;i++)
  {
    result[i]=inter1[i];
    result[i+inter1.size()-1]=inter2[i];
  }

  return result;
}

int main(int argc, char const *argv[]) {
  const string wcFile = "/home/christine/catkin_ws/src/robot_plugin/WorkCell/WRS-RobWork/WRS_v3.wc.xml";
	const string deviceName = "UR10A";
  const string device2name= "UR10B";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	device1 = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 1;
	}
  device2 = wc->findDevice(device2Name);
  if (device == NULL) {
    cerr << "Device: " << deviceName << " not found!" << endl;
    return 2;
  }
  state = wc->getDefaultState();
  Q pose1();
	device1->setQ(pose, state);

  while (ros::ok()) {

  }


  return 0;
}
