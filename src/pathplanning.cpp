#include <iostream>
#include <iomanip> // std::setprecision
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>

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

#define MAXTIME 120.

const bool debug    = false;
const bool saveLua  = false;

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
    //const string wcFile = "/home/exchizz/SDU/Skole/7.Semester/ROVI/Robots/PF1/Kr16WallWorkCell/Scene.wc.xml"; // Nerup WorkCell location
    //const string wcFile = "/media/sf_VM-share/Kr16WallWorkCell/Scene.wc.xml"; // Lundby WorkCell location
    const string wcFile = "../Kr16WallWorkCell/Scene.wc.xml"; // Relative path
    const string deviceName = "KukaKr16";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

    rw::math::Math::seed();

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

    // Define a frame for the bottle and the tool
    const string BottleName = "Bottle";
    const string ToolName = "Tool";
    Frame* BottleFrame = wc->findFrame(BottleName);
    Frame* ToolFrame = wc->findFrame(ToolName);
    if (BottleFrame == NULL) {
        cerr << "Device: " << BottleName << " not found!" << endl;
        return 0;
    }
    if (ToolFrame == NULL) {
        cerr << "Device: " << ToolName << " not found!" << endl;
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

    int iteration_ctr = 600;

    for (double extend = 0.005*(iteration_ctr+1); extend <= 6.005; extend+=0.005) { // epsilon range
        ofstream data;
        data.open ("data" + to_string(iteration_ctr) + ".csv");
        data << "extend,i,time,steps,path_length\n";
        cout << "Epsilon = " << extend << endl;
        for (size_t h = 0; h < 100; h++) { // Samples per epsilon
            // if (h%10 == 0)
                cout << "Iteration = " << h << endl;
            QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
            QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
            //double extend = 0.1;
            QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
            if (!checkCollisions(device, state, detector, from))
                return 0;
            if (!checkCollisions(device, state, detector, to))
                return 0;

            // Prepare stuff for path length
            PathAnalyzer path_analyser(device,state);
            PathAnalyzer::CartesianAnalysis path_length;

            if (debug)
                cout << "Planning from " << from << " to " << to << endl;
            QPath path;
            Timer t;
            t.resetAndResume();
            planner->query(from,to,path,MAXTIME);
            t.pause();
            if (debug)
                cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
            if (t.getTime() >= MAXTIME) {
                if (debug)
                    cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
            }
            // Calc the path length
            path_length = path_analyser.analyzeCartesian(path,ToolFrame);
            double total_path_length = path_length.length;
            if (debug)
                cout << "Path length is: " << total_path_length << endl;



	    // Zum'a dat' nizzle shizzle
	    double sum = 0;
	    for (QPath::iterator it = path.begin(); it < path.end()-1; it++) {
		Q diff = *(it+1) - *it;
		sum += diff.norm2();
	    }


	    cout << "Sum : " <<  sum << std::endl;
	    // Zum'a dat' nizzle shizzle - end

            data << setprecision(8) << extend << "," << h << "," << t.getTime() << "," << path.size() << "," << total_path_length << "\n";
            // Saving path in LUA format for the RobWork simulator
            if (saveLua) {
                ofstream myfile;
                myfile.open ("run.lua");

                myfile << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
                myfile << "state = wc:getDefaultState()" << endl;
                myfile << "device = wc:findDevice(\"KukaKr16\")" << endl;
                myfile << "gripper = wc:findFrame(\"Tool\")" << endl;
                myfile << "bottle = wc:findFrame(\"Bottle\")" << endl;
                myfile << "table = wc:findFrame(\"Table\")" << endl;

                myfile << "function setQ(q)" << endl;
                myfile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])" << endl;
                myfile << "device:setQ(qq,state)" << endl;
                myfile << "rws.getRobWorkStudio():setState(state)" << endl;
                myfile << "rw.sleep(0.1)" << endl;
                myfile << "end" << endl;

                myfile << "function attach(obj, tool)" << endl;
                myfile << "rw.gripFrame(obj, tool, state)" << endl;
                myfile << "rws.getRobWorkStudio():setState(state)" << endl;
                myfile << "rw.sleep(0.1)" << endl;
                myfile << "end" << endl;

                stringstream ss;
                string ssString;
                int i = 0;

                for (QPath::iterator it = path.begin(); it < path.end(); it++) {
                    if(i++ == 1)
                        myfile << "attach(bottle,gripper)" << endl;

                    ss << *it;
                    ssString = ss.str();
                    ssString =  ssString.substr(4) ;
                    myfile << "setQ(" <<  ssString << ")" << endl;
                    ss.str("");
                }
                myfile << "attach(bottle,table)" << endl;
            }
        }
        data.close();
        iteration_ctr++;
    }


    if (debug)
        cout << "Program done." << endl;
    return 0;
}
