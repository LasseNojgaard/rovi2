//------------------
// INCLUDES
//------------------


// RobWork
#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common.hpp>

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



//--------------------
// Functions
//--------------------





//----------
// Main
//---------
int main()
{
    // Define Workcell path and Robot name
    string wcFile = "../Workcells/WRS-RobWork/WRS_v3.wc.xml";
    string deviceAName = "UR10A";
    string deviceBName = "UR10B";
    std::cout << "Trying to use workcell: " << wcFile << std::endl;
    std::cout << "with device(s): " << deviceAName << std::endl;
    std::cout << "and device(s): " << deviceBName << std::endl;

    string wcFileUR5 = "../Workcells/UR5/Scene.wc.xml";

    // Load Workcell
    WorkCell::Ptr workcell = rw::loaders::WorkCellLoader::Factory::load(wcFile);

    // Load Robots
    Device::Ptr deviceA = workcell->findDevice(deviceAName);
    Device::Ptr deviceB = workcell->findDevice(deviceBName);
    if (deviceA == NULL && deviceB == NULL)
    {
        std::cerr << "Device: " << deviceAName << " and device: " << deviceBName << " not found!" << std::endl;
        return -1;
    }
    else if(deviceA != NULL && deviceB == NULL)
    {
        std::cerr << "Device: " << deviceAName << " found!" << std::endl;
        std::cerr << "Device: " << deviceBName << " not found!" << std::endl;
        return -1;
    }
    else if(deviceA == NULL && deviceB != NULL)
    {
        std::cerr << "Device: " << deviceAName << " not found!" << std::endl;
        std::cerr << "Device: " << deviceBName << " found!" << std::endl;
        return -1;
    }
    else
    {
        std::cerr << "Device: " << deviceAName << " and device: " << deviceBName << " found!" << std::endl;
    }
    // Get the state of the workcell
    cout << "Getting state of workcell" << endl;
    State state = workcell->getDefaultState();

    // Create a default collision detector
    CollisionDetector::Ptr detector = new CollisionDetector(workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    // Create a configuration
    cout << "Collecting configuration data" << endl;
    Q qMinA = deviceA->getBounds().first;
    Q qMaxA = deviceA->getBounds().second;
    Q qMinB = deviceB->getBounds().first;
    Q qMaxB = deviceB->getBounds().second;
    Math::seed();

    Q tempQA = Math::ranQ(qMinA,qMaxA);
    Q tempQB = Math::ranQ(qMinB, qMaxB);
    std::cout << "Random configuration A: " << tempQA << std::endl;
    std::cout << "Random configuration B: " << tempQB << std::endl;

    // Set robot to configuration, IMPORTANT!
    deviceA->setQ(tempQA, state);
    deviceB->setQ(tempQB, state);

    // Test configuration for collision
    cout << "Testing for collition" << endl;
    CollisionDetector::QueryResult data;

    bool collision = detector->inCollision(state,&data);

    cout << "Conf. in collision: " << collision << endl;





    // Test for SBL
    cout << "Setting up SBL" << endl;
    // Constraint
    rw::pathplanning::QConstraint::Ptr ConstraintA = rw::pathplanning::QConstraint::make(detector, deviceA, state);
    rw::pathplanning::QConstraint::Ptr constraintB = rw::pathplanning::QConstraint::make(detector, deviceB, state);



    CollisionDetector coldect(workcell, ProximityStrategyYaobi::make());

    QConstraint::Ptr constraint = QConstraint::make(&coldect, deviceA, state);

    QEdgeConstraintIncremental::Ptr edgeconstraint = QEdgeConstraintIncremental::makeDefault(ConstraintA, deviceA);

    QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(SBLSetup::make(ConstraintA, edgeconstraint, deviceA, 0.01, 0.01));

    QSampler::Ptr cfreeQ = QSampler::makeConstrained(QSampler::makeUniform(deviceA), ConstraintA, 100000);
    Q pos = deviceA->getQ(state);
    cout << "Making the trajectory" << endl;
    rw::trajectory::Path<Q> path;

    int maxCount = 10;
    cout << "Begin" << endl;
    for(int cnt = 0; cnt < maxCount; cnt++)
    {
      cout << "Making nextQ " << endl;
      const Q next = cfreeQ->sample();
      cout << "Checking the nextQ" << endl;
      const bool ok = planner->query(pos,next,path);
      cout << "Done checking nextQ" << endl;
      if(!ok)
      {
        std::cout << "Path " << cnt << " not found.\n" ;
        return 0;
      }
      else
      {
        cout << "here" << endl;
          pos = next;
      }
      cout << "cnt: " << cnt << " / " << maxCount << endl;
    }

    cout << "End" << endl;

    const std::vector<State> states = Models::getStatePath(*deviceA, path, state);

    PathLoader::storeVelocityTimedStatePath(
        *workcell, states, "ex-path-planning.rwplay");




    cout << "Done" << endl;

    return 0;


}
