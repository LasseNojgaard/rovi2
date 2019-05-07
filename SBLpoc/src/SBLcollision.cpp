
//------------------
// INCLUDES

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
#include <rw/models/TreeDevice.hpp>

#include <fstream>
#include <iostream>
#include <time.h>

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

//--------------------
// Functions

rw::trajectory::Path<Q> SBL(double maxDistance, Q inputConfiguration)
{
  rw::trajectory::Path<Q> somePath;

  return somePath;
}

void SBLTest(double epsilon, int amount, int iterations, double increment, State theState, int randomMode)
{
  if(randomMode)
  {
    Math::seed(randomMode)
  }
  else
  {
    Math::seed()
  }

  string date = "Tue";
  ofstream somefile;
  somefile.open(date.append(".txt"));

  Q pos = deviceTree->getQ(theState);

  Q minQ = deviceTree->getBounds().first()
  Q maxQ = deviceTree->getBounds().second()

  int counter = 0;

  somefile << "date \n";
  for(int j = 0; j < iterations; j++)
  {
    epsilon += increment;
    for(int i = 0; i < amount; i++)
    {
      cout << "Setting up SBL and starting timer" << endl;
      timeStart = clock();
      // Constraints
      rw::pathplanning::QConstraint::Ptr ConstraintA = rw::pathplanning::QConstraint::make(detector, deviceTree, theState);

      CollisionDetector coldect(workcell, ProximityStrategyYaobi::make());

      QConstraint::Ptr constraint = QConstraint::make(&coldect, deviceTree, theState);

      QEdgeConstraintIncremental::Ptr edgeconstraint = QEdgeConstraintIncremental::makeDefault(ConstraintA, deviceTree);

      QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(SBLSetup::make(ConstraintA, edgeconstraint, deviceTree, epsilon, epsilon));

      QSampler::Ptr cfreeQ = QSampler::makeConstrained(QSampler::makeUniform(deviceTree), ConstraintA, 100000);

      Q tempQ = Math::ranQ(minQ, maxQ);

      Q endPos = tempQ;
      cout << "pos: " << pos << endl;
      cout << "endPos" << endPos << endl;
      cout << "Making the trajectory" << endl;

      rw::trajectory::Path<Q> path;

      cout << "Begin" << endl;

      cout << "Next configuration: " << endPointQ << endl;

      cout << "Reaching down the rabbithole" << endl;
      const bool ok = planner->query(pos,endPointQ,path);
      cout << "Done checking nextQ" << endl;
      if(!ok)
      {
        std::cout << "Path " << "0" << " not found.\n" ;
      }
      else
      {
        cout << "Path found" << endl;
      }

      cout << "End" << endl;

      double timeUsed = ((double)(clock() - timeStart)) / CLOCKS_PER_SEC;

      const std::vector<State> states = Models::getStatePath(*deviceTree, path, state);

      cout << "Time used in seconds: " << timeUsed << endl;
      cout << "Write out path: " << endl;
      cout << "length of path: " << path.size() << endl;

      somefile << timeUsed << " ; " << path.size() << " ; " << epsilon " ; " << amount;

      cout << counter+1 << " / " << amount*iterations << endl;
      counter++;
    }

    somefile.close();
    cout << "Done" << endl;
  }
}


//----------
// Main
//---------
int main()
{
    // Define Workcell path and Robot name
    //string wcFile = "../Workcells/WRS-RobWork12DOG/WRS_v3v2.wc.xml"; //Our own workcell and own 12 degree robot
    //string wcFile = "../Workcells/UR5/Scene.wc.xml";
    string wcFile = "../Workcells/WRS-RobWork/WRS_v3.wc.xml";

    //--- Device names:
    //string deviceAName = "UR5";
    string deviceAName = "UR10A";
    string deviceBName = "UR10B";

    std::cout << "Trying to use workcell: " << wcFile << std::endl;
    std::cout << "with device(s): " << deviceAName << " with device(s): " << deviceBName << std::endl;


    // Load Workcell
    WorkCell::Ptr workcell = rw::loaders::WorkCellLoader::Factory::load(wcFile);

    // Load Robots
    Device::Ptr deviceA = workcell->findDevice(deviceAName);
    Device::Ptr deviceB = workcell->findDevice(deviceBName);

    /*if(deviceA == NULL)
    {
        std::cerr << "Device: " << deviceAName << " not found!" << std::endl;
    }
    else
    {
        std::cerr << "Device: " << deviceAName << " found!" << std::endl;
    }*/
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

    CollisionDetector::QueryResult data;

    // Create a configuration
    cout << "Collecting configuration data" << endl;
    //Q qMinA = deviceA->getBounds().first;
    //Q qMaxA = deviceA->getBounds().second;
    /*Q qMinB = deviceB->getBounds().first;
    Q qMaxB = deviceB->getBounds().second;*/
    Math::seed();

    cout << "Making the vectorOfEnds" << endl;
    vector<kinematics::Frame*> vectorOfEnds;
    vectorOfEnds.push_back(deviceA->getEnd());
    vectorOfEnds.push_back(deviceB->getEnd());

    string refFrame = "RefFrame";
    string TopPlateName = "TopPlate";
    cout << "making tree" << endl;
    TreeDevice::Ptr deviceTree = new TreeDevice(workcell->findFrame(refFrame), vectorOfEnds, "TreeDevice", state);

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

    cout << "Euclidian: " << tempQA.norm2() - tempQB.norm2() << endl;

    //--Roadmap
    //--TreeStructure

    //--PathTesting

    //--Path trimming

    //--Optimization
*/

    //----------------------------

    // Test for SBL
    cout << "Setting up SBL" << endl;

    timeStart = clock();
    // Constraint
    rw::pathplanning::QConstraint::Ptr ConstraintA = rw::pathplanning::QConstraint::make(detector, deviceTree, state);
    //rw::pathplanning::QConstraint::Ptr constraintB = rw::pathplanning::QConstraint::make(detector, deviceB, state);

    CollisionDetector coldect(workcell, ProximityStrategyYaobi::make());
    //PKG , Check source fil for link fra yaobi.
    // Interpolate path includeret i robwork
    //CollisionDetector coldect(workcell, ProximityStrategyFactory::makeDefault());

    QConstraint::Ptr constraint = QConstraint::make(&coldect, deviceTree, state);

    QEdgeConstraintIncremental::Ptr edgeconstraint = QEdgeConstraintIncremental::makeDefault(ConstraintA, deviceTree);

    double eps = 0.4;

    QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(SBLSetup::make(ConstraintA, edgeconstraint, deviceTree, eps, eps));

    QSampler::Ptr cfreeQ = QSampler::makeConstrained(QSampler::makeUniform(deviceTree), ConstraintA, 100000);

    Q pos = deviceTree->getQ(state);

    cout << "pos: " << pos << endl;
    cout << "Making the trajectory" << endl;
    Q endPointQ = Q(12);

    vector<double> configurationVector = {3.407494068145752, -1.9994360409178675, -1.8003206253051758,
                                          1.8201419550129394, -0.6341050306903284, -5.036266628895895,
                                          -0.35499412218202764, -2.6603156528868617, 2.1026347319232386,
                                          0.20941845952954097, 0.6679062843322754, 1.9573044776916504};
    for(int i = 0; i <12 ; i++)
    {
      endPointQ[i] = configurationVector[i];
    }

    rw::trajectory::Path<Q> path;

    //int maxCount = 5;

    cout << "Begin" << endl;

    cout << "Next configuration: " << endPointQ << endl;

    cout << "Reaching down the rabbithole" << endl;
    const bool ok = planner->query(pos,endPointQ,path);
    cout << "Done checking nextQ" << endl;
    if(!ok)
    {
      std::cout << "Path " << "0" << " not found.\n" ;
    }
    else
    {
      cout << "Path found" << endl;
    }

    /*
    for(int cnt = 0; cnt < maxCount; cnt++)
    {
      cout << "Making nextQ " << endl;
      const Q next = (pos+incrementation); cfreeQ->sample();

      cout << "Next configuration: " << next << endl;

      cout << "Checking the nextQ" << endl;
      const bool ok = planner->query(pos,next,path,60);
      cout << "Done checking nextQ" << endl;
      if(!ok)
      {
        std::cout << "Path " << cnt << " not found.\n" ;
      }
      else
      {
        cout << "Path found" << endl;
          pos = next;
      }
      cout << "cnt: " << cnt+1 << " / " << maxCount << " ------------------" <<endl;
    }
*/

    cout << "End" << endl;

    const std::vector<State> states = Models::getStatePath(*deviceTree, path, state);

    PathLoader::storeVelocityTimedStatePath(
        *workcell, states, "ex-path-planning.rwplay");

    double timeUsed = ((double)(clock() - timeStart)) / CLOCKS_PER_SEC;

    cout << "Time used in seconds: " << timeUsed << endl;
    cout << "Write out path: " << endl;
    cout << "length of path: " << path.size() << endl;

    cout << path[2].norm2()-path[3].norm2() << endl;
/*
    for(size_t i = 0; i < path.size(); i++ )
    {
      cout << path[i] << endl;
    }
*/

    string date = "Tue";
    ofstream somefile;
    somefile.open(date.append(".txt"));

    somefile<< "Test test \n";
    somefile<< timeUsed << " , " << path.size();
    somefile.close();


    cout << "Done" << endl;

    return 0;
}


// TODO ----------------------

/*
    - Check the time needed to trim the paths and see how much time it cuts.
      Check and see wether it is recommended or not.
    - Make 10 paths and use the shortest of these to do the moves.
    - Check the interpolation
*/
