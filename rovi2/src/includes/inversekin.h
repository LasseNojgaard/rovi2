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
#define tcpdisp 0.25


rw::math::Q createQ12(std::vector<rw::math::Q> iter1;,std::vector<rw::math::Q> iter2, int index1, int index2){
    rw::math::Q result;
    for(int i=0;i<result.size()/2;i++) {
        result[i] = iter1[index1][i];
        result[i + result.size()/2 - 1] = iter2[index2][i];
    }
    return result
}

rw::math::Transform3D<> relatePosetoBase(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device ,rw::math::Transform3D<> pose, rw::kinematics::State state){
    rw::kinematics::Frame* table=wc->findFrame("TopPlate");
    rw::kinematics::Frame* base=device->getEnd();
    rw::math::Transform3D<> table2base= table->fTf(base,state);
    return pose*table2base;
}

rw::math::Transform3D<> accountForGripper(rw::math::Transform3D pose){
  rw::math::Vector3D translation=pose.P();
  translation[z]+=tcpdisp
  return rw::math::Transform3D<>(translation,pose.R())
}
std::vector<std::vector<rw::math::Q>> findGoalConfig(rw::models::Device::Ptr device_1,rw::models::Device::Ptr device_2, rw::kinematics::State state, rw::math::Transform3D<> pose1, rw::math::Transform3D<> pose2,bool tried=false) {


    std::vector<std::vector<rw::math::Q>> result
    pose1=accountForGripper(pose1);
    rw::invkin::IterativeIK::Ptr solver=rw::invkin::JacobianIKSolver::makeDefault(device_1,state);
    std::vector<rw::math::Q> iter1= solver->solve(pose1,state);
    pose2=accountForGripper(pose2)
    rw::invkin::IterativeIK::Ptr solver2=rw::invkin::JacobianIKSolver::makeDefault(device_2,state);
    std::vector<rw::math::Q> iter2 = solver2->solve(pose2,state);

    if (iter1.size()==0 || iter2.size()==0){
        if (!tried)
            result=findGoalConfig(device_1, device_2, state, pose2, pose1, true);
        return result;
      }
      result.push_back(iter1);
      result.push_back(iter2);
      return result;
}
