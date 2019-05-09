
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

rw::trajectory::Path<Q> SBL(Q inputConfiguration, Q outputConfiguration, CollisionDetector::Ptr aDetector, TreeDevice::Ptr aDeviceTree, State aState, double epsilon, WorkCell::Ptr aWorkcell)
{
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
  cout << inputConfiguration << endl;

  cout << "Next configuration: " << endl;
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

  cout << "End" << endl;

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
  while (i < aPath.size()-2)
  {
    bool collision = extBinarySearch(eps, tempPath[i], tempPath[i+2], theState, theDetector, theDevice);
    cout << "Object:" << i << " and " << i+2 << "maybe in collision: " << collision << endl;
    if (collision == 0)
    {
      aPath.erase(aPath.begin()+i+1);
      if (i>0)
      {
        i -= 1;
      }
      else
      {
        i += 1;
      }
    }
  }

  return aPath;
}

rw::trajectory::Path<Q> interpolate()
{
  rw::trajectory::Path<Q> tempPath;



  return tempPath;
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

    Math::seed();

    cout << "Making the vectorOfEnds" << endl;
    vector<kinematics::Frame*> vectorOfEnds;
    vectorOfEnds.push_back(deviceA->getEnd());
    vectorOfEnds.push_back(deviceB->getEnd());

    string refFrame = "RefFrame";
    string TopPlateName = "TopPlate";
    cout << "making tree" << endl;
    TreeDevice::Ptr deviceTree = new TreeDevice(workcell->findFrame(refFrame), vectorOfEnds, "TreeDevice", state);

    //----------------------------



    Q positionBegin = deviceTree->getQ(state);

    Q endPointQ = Q(12);

    vector<double> configurationVector = {3.407494068145752, -1.9994360409178675, -1.8003206253051758,
                                          1.8201419550129394, -0.6341050306903284, -5.036266628895895,
                                          -0.35499412218202764, -2.6603156528868617, 2.1026347319232386,
                                          0.20941845952954097, 0.6679062843322754, 1.9573044776916504};
    for(int i = 0; i <12 ; i++)
    {
      endPointQ[i] = configurationVector[i];
    }
    double anEpsilon = 0.2;

    timeStart = clock();
    rw::trajectory::Path<Q> goodPath = SBL(positionBegin, endPointQ, detector, deviceTree, state, anEpsilon, workcell);
    double timeUsed = ((double)(clock() - timeStart)) / CLOCKS_PER_SEC;

    cout << "length of path: " << goodPath.size() << endl;
    for(size_t i = 0 ; i < goodPath.size() ; i++)
    {
      //cout << goodPath[i] << endl;
    }

    //--Path trimming
    timeStart = clock();
    rw::trajectory::Path<Q> prunedPath = pathPrune(goodPath, anEpsilon, state, detector, deviceTree);
    double timeUsedPrune = ((double)(clock() - timeStart)) / CLOCKS_PER_SEC;
    cout << "length of path: " << prunedPath.size() << endl;
    for (size_t i = 0; i < prunedPath.size(); i++)
    {
      //cout << prunedPath[i] << endl;
    }




    cout << "Time used in seconds on SBL: " << timeUsed << endl;
    cout << "Write out path: " << endl;
    cout << "length of path: " << goodPath.size() << endl;


    cout << "Time used in seconds on pruning: " << timeUsedPrune << endl;
    cout << "Write out path: " << endl;
    cout << "length of path: " << prunedPath.size() << endl;

    const std::vector<State> states = Models::getStatePath(*deviceTree, goodPath, state);

    const std::vector<State> states2 = Models::getStatePath(*deviceTree, prunedPath, state);



    PathLoader::storeVelocityTimedStatePath(
        *workcell, states, "ex-path-planning.rwplay");

    PathLoader::storeVelocityTimedStatePath(*workcell, states2, "ex-path-pruned-planning.rwplay");
        //--PathTesting


        //--Optimization



    cout << "-----------------------------------------------------------------" << endl;

    Q testMin = {3, -1, -1, -1};
    Q testMax = {3, 1, 1, 1};

    Q test2Min = {3, -1, -1, -1};
    Q test2Max = {3, 1, 1, 1};

    Q testres = Math::ranQ(testMin, testMax);
    Q test2res = Math::ranQ(test2Min, test2Max);

    cout << "Position norm 2: " << positionBegin.norm2() << endl;

    cout << "EndpointQ norm 2: " << endPointQ.norm2() << endl;

    cout << "Difference in norm2: " << positionBegin.norm2()-endPointQ.norm2() << endl;

    cout << "Position norm 2:  " << testres.norm2() << endl;

    cout << "EndpointQ norm 2: " << test2res.norm2() << endl;

    cout << "Difference in norm2: " << testres.norm2()-test2res.norm2() << endl;
    cout << "Real difference: " << abs(testres.norm2()-test2res.norm2()) << endl;

    //double testnorm2 = testres.norm2();
    //double test2norm2 = test2res.norm2();
    double separation = 100;
    double distance = abs(testres.norm2()-test2res.norm2());

    double factor = distance*separation;

    Q increment = (testres-test2res)/factor;


    cout << "Increment" << increment.norm2() << endl;

    cout << "Difference " << (testres.norm2()-test2res.norm2())/increment.norm2() << endl;

    cout << factor << endl;



    string date = "Thu";
    ofstream somefile;
    somefile.open(date.append(".txt"));

    somefile<< "Test test \n";
    somefile<< timeUsed << " , " << prunedPath.size();
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
