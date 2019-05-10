#include <iostream>
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
#include <rw/rw.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <fstream>










void findBestSolution(rw::math::Q current1, rw::math::Q current2,std::vector<rw::math::Q> iter1,std::vector<rw::math::Q> iter2, rw::math::Q &result){
    int leastScore=0;
    //std::cout << "!!!" << std::endl;
    //std::cout << iter1[0] << std::endl;
    //std::cout << iter2[0] << std::endl;
    for(int i=0;i<result.size()/2;i++) {
        result[i] = iter1[leastScore][i];
        result[i + result.size()/2 - 1] = iter2[leastScore][i];
    }
   // std::cout << result << std::endl;
}
rw::math::Transform3D<> relatePosetoBase(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device ,rw::math::Transform3D<> pose, rw::kinematics::State state){
    rw::kinematics::Frame* table=wc->findFrame("TopPlate");
    rw::kinematics::Frame* base=device->getBase();
    rw::math::Transform3D<> table2base= table->fTf(base,state);
    return pose*table2base;
}
rw::math::Q findGoalConfig(rw::models::Device::Ptr device_1,rw::models::Device::Ptr device_2, rw::kinematics::State state, rw::math::Transform3D<> pose1, rw::math::Transform3D<> pose2,bool tried=false) {

    rw::math::Q result(12);
    rw::invkin::IterativeIK::Ptr solver=rw::invkin::JacobianIKSolver::makeDefault(device_1,state);
    std::vector<rw::math::Q> iter1= solver->solve(pose1,state);
    rw::invkin::IterativeIK::Ptr solver2=rw::invkin::JacobianIKSolver::makeDefault(device_2,state);
    std::vector<rw::math::Q> iter2 = solver2->solve(pose2,state);

    if (iter1.size()==0 || iter2.size()==0) {
        if (!tried)
            result=findGoalConfig(device_1, device_2, state, pose2, pose1, true);
        return result;
    }
    else
    findBestSolution(device_1->getQ(state),device_2->getQ(state),iter1, iter2, result);
    std::cout << "FOUND!" << std::endl;
        return result;
}


int main() {
    const std::string wcFile = "/home/christine/catkin_ws/src/robot_plugin/WorkCell/WRS-RobWork/WRS_v3.wc.xml";
    const std::string deviceName = "UR10B";
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    rw::models::Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 1;
    }

    rw::math::Transform3D<> pose(rw::math::Vector3D<>(0,0,0),rw::math::Rotation3D<>(1,0,0,0,1,0,0,0,1));
    rw::math::Transform3D<> result(rw::math::Vector3D<>(1,1,1),rw::math::Rotation3D<>(2,2,2,2,2,2,2,2,2));
    result=pose*result;
    std::cout << result.R() << std::endl << result.P();

    result=relatePosetoBase(wc,device ,pose, wc->getDefaultState());
    std::cout << std::endl << std::endl << result.R() << std::endl << result.P() << std::endl;

    rw::kinematics::State state= wc->getDefaultState();
    rw::math::Q home=device->getQ(state);
    device->setQ(rw::math::Q(6,-0.022, -2.746, 0.798, -2.042, -1.574, 1.2),state);
    result= device->baseTend(state);
    std::cout << std::endl << "--------------------------"<< std::endl << result.R() << std::endl << result.P() << std::endl;
    device->setQ(home,state);
    rw::math::Q intgoal=findGoalConfig(device,device,state,result,result);
    rw::math::Q goal(6);
    //std::cout << goal << std::endl;
    //std::cout << intgoal << std::endl;
    goal[0]=intgoal[0];goal[1]=intgoal[1];goal[2]=intgoal[2];goal[3]=intgoal[3];goal[4]=intgoal[4];goal[5]=intgoal[5];
    //std::cout << goal << std::endl;
    device->setQ(goal,state);
    result= device->baseTend(state);
    std::cout << result.R() << std::endl << result.P() << std::endl;
    std::cout<< "End!" <<std::endl;

	return 0;
}