#include <iostream>
#include <fstream>
#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>

#include "../../PRM/TrajectoryPlanner.h"
#include "../../Grasp/PlanBase.h"
#include "../../Grasp/PlanInterface.h"
#include "../../MotionFile/MotionFileControl.h"
#include "../../MotionFile/MotionFileBar.h"

using namespace std;
using namespace cnoid;
using namespace grasp;


namespace py = pybind11;

namespace {
int add (int j) {
	int i = 1994;
	// i = TrajectoryPlanner::testPlanning();
	// TrajectoryPlanner::doTrajectoryPlanning();
	return i + j;
}

void Plan_motion() {
	if (! PlanBase::instance()->targetArmFinger) {
		cnoid::MessageView::instance()->cout() << "Error: set robot" << endl;
	}
	TrajectoryPlanner * trajectoryPlanner_;
	trajectoryPlanner_ = new TrajectoryPlanner();
	cnoid::MessageView::instance()->cout()<< "load class success! " << endl;
	bool ret = trajectoryPlanner_->doTrajectoryPlanning();
	if (ret)
		cnoid::MessageView::instance()->cout() << "Trajectory Planning is finished" << endl;
		
	else 
		cnoid::MessageView::instance()->cout() << "Trajectory Planning is failed" << endl;

}
bool Load_motion_file() {
	
	static MotionFileControl* instance = new MotionFileControl();
	string motionfile;
	motionfile = "./ext/bpbot/data/motion/motion.dat";
	FILE *fp;
	bool success;
	if( (fp = fopen((motionfile).c_str(), "r")) != NULL ){
		success = MotionFileControl::instance()->LoadFromMotionFile(motionfile);
		fclose(fp);
	}
	cnoid::MessageView::instance()->cout() << "Loaded motion file: " << success << endl;

	return success;
}
void Get_motion() {
	PlanBase * tc = PlanBase::instance();
	int size = tc->graspMotionSeq.size();
	cnoid::MessageView::instance()->cout() << "[ xinyi test ] Size is " << size << endl;
	for (int i; i < size; i++) {
		cout << "motionTime: " << tc->graspMotionSeq[i].motionTime << endl;
	}
}
}

namespace cnoid {
void exportPlanning(py::module m) {

	m.doc() = "pybind11 motion planning";
	m.def("add", &add);
	m.def("plan", &Plan_motion);
	m.def("load_file", &Load_motion_file);
	m.def("get_motion", &Get_motion);
}
}