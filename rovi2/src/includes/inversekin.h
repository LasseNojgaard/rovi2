#include <iostream>
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics.hpp>
#include <rw/rw.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

#define z 2
#define qlength 12
#define tcpdisp 0.75
#define MAX 10000


rw::math::Q createQ12(std::vector<rw::math::Q> iter1, std::vector<rw::math::Q> iter2, int index1, int index2){
    rw::math::Q result(qlength);
    for(int i=0;i<result.size()/2;i++) {
        result[i] = iter1[index1][i];
        result[i + result.size()/2] = iter2[index2][i];
    }
    return result;
}

rw::math::Transform3D<> relatePoseToBase(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device ,rw::math::Transform3D<> pose, rw::kinematics::State state){
  rw::kinematics::Frame* table=wc->findFrame("TopPlate");
  rw::kinematics::Frame* base=device->getBase();
  rw::math::Transform3D<> table2base= table->fTf(base,state);
  return pose*table2base;
}

rw::math::Transform3D<> accountForGripper(rw::math::Transform3D<> pose)
{
  rw::math::Vector3D<> translation=pose.P();
  translation[z]+=tcpdisp;
  return rw::math::Transform3D<>(translation,pose.R());
}
std::vector<std::vector<rw::math::Q>> findGoalConfig(rw::models::WorkCell::Ptr wc,rw::models::Device::Ptr device_1,rw::models::Device::Ptr device_2, rw::kinematics::State state, rw::math::Transform3D<> pose1, rw::math::Transform3D<> pose2,bool &tried) {


    std::vector<std::vector<rw::math::Q>> result;
    rw::models::Device::QBox bounds1=device_1->getBounds();
    rw::models::Device::QBox boundsAdefault=bounds1;
    bounds1.first[1]=0;
    bounds1.second[1]=rw::math::Pi;
    device_1->setBounds(bounds1);
    rw::models::Device::CPtr device1=&*device_1;

    rw::invkin::JacobianIKSolver solver(device1,state);
    solver.setClampToBounds(true);
    solver.setMaxIterations(MAX);
    pose1=relatePoseToBase(wc, device_1,pose1,state);
    pose1=accountForGripper(pose1);
    std::vector<rw::math::Q> iter1= solver.solve(pose1,state);
    device_1->setBounds(boundsAdefault);

    rw::models::Device::QBox bounds2=device_2->getBounds();
    rw::models::Device::QBox boundsBDefault=bounds2;
    bounds2.first[1]=-rw::math::Pi;
    bounds2.second[1]=0;
    device_2->setBounds(bounds2);
    rw::models::Device::CPtr device2=&*device_2;


    pose2=relatePoseToBase(wc, device_2,pose2,state);
    pose2=accountForGripper(pose2);
    rw::invkin::JacobianIKSolver solver2(device_2,state);
    std::vector<rw::math::Q> iter2 = solver2.solve(pose2,state);
    device_2->setBounds(boundsBDefault);


    if (iter1.size()==0 || iter2.size()==0){
        if (!tried)
	  {
	    tried = true;
            result=findGoalConfig(wc,device_1, device_2, state, pose2, pose1, tried);
	   }
        return result;

      }
      result.push_back(iter1);
      result.push_back(iter2);
      return result;
}
