#include <iostream>
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
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;
 
#define MAXTIME 10.
 
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}
 
int main(int argc, char** argv) {
    const string wcFile = "/home/exchizz/SDU/Skole/7.Semester/ROVI/Robots/PF1/Kr16WallWorkCell/Scene.wc.xml";
    const string deviceName = "KukaKr16";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
 
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }
    State state = wc->getDefaultState();



    Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
    Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);

    device->setQ(from,state);

    Frame* BottleFrame = wc->findFrame("Bottle");
    Frame* ToolFrame = wc->findFrame("Tool");


    if (ToolFrame == NULL) {
        cerr << "Device: \"Tool\" not found!" << endl;
        return 0;
    }

    if (BottleFrame == NULL) {
        cerr << "Device: \"Bottle\" not found!" << endl;
        return 0;
    }


    Kinematics::gripFrame(BottleFrame, ToolFrame, state);

    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    /** Most easy way: uses default parameters based on given device
        sampler: QSampler::makeUniform(device)
        metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
        extend: 0.05 */
    //QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);



    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    double extend = 0.1;
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
 
    //Q from(6,-0.2,-0.6,1.5,0.0,0.6,1.2);
    //Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner
    //Q to(6,1.4,-1.3,1.5,0.3,1.3,1.6);
    //q_pick = (-3.142,-0.827,-3.002,-3.143,0.099,-1.573) [rad]
    //q_place = (1.571,0.006,0.030,0.153,0.762,4.490) [rad]
    //Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner



    if (!checkCollisions(device, state, detector, from))
        return 0;
    if (!checkCollisions(device, state, detector, to))
        return 0;
    cout << "Planning from " << from << " to " << to << endl;
    QPath path;
    Timer t;
    t.resetAndResume();
    planner->query(from,to,path,MAXTIME);
    t.pause();
    cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
    if (t.getTime() >= MAXTIME) {
        cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
    }

    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        cout << *it << endl;
    }

    cout << "Program done." << endl;
    return 0;
}
