#include <iostream>
#include <fstream>
#include <math.h>
#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>

#include "../../PRM/TrajectoryPlanner.h"
#include "../../Grasp/PlanBase.h"
#include "../../Grasp/PlanInterface.h"
#include "../../MotionFile/MotionFileControl.h"
#include "../../MotionFile/MotionFileBar.h"
#include <pybind11/stl.h> // vector
#include <pybind11/operators.h>//operator
using namespace std;
using namespace cnoid;
using namespace grasp;


namespace py = pybind11;

namespace {
int add (int j) {
	int i = 1994;
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
vector<bool> Load_motion_file(string motionfilename, bool dual_arm=false) {
//bool Load_motion_file(string motionfilename) {
	
	MotionFileControl* instance = new MotionFileControl();
	FILE *fp;
	if( (fp = fopen((motionfilename).c_str(), "r")) != NULL ){
		instance->LoadFromMotionFile(motionfilename, dual_arm);
		fclose(fp);
	}
	vector<bool> success = instance->isMotionFileSucceed;
	return success;
}
vector<double> Get_motion() {
	vector<double> seqs;
	PlanBase * tc = PlanBase::instance();
	int size = tc->graspMotionSeq.size();

	//int stateLft_old = tc->NOT_GRASPING;

	for (int i; i < size; i++) {
		// start to create motion sequence
		seqs.push_back(tc->graspMotionSeq[i].motionTime);

		//int gstateLft = tc->graspMotionSeq[i].graspingState2;
		//// 0: close , 1: open, 2. nothing, stay same
		//if (gstateLft == tc->GRASPING && stateLft_old == tc->UNDER_GRASPING) 
		//	seqs.push_back(0);
		//else if (gstateLft == tc->NOT_GRASPING && stateLft_old == tc->UNDER_GRASPING) 
		//	seqs.push_back(1);
		//else if (gstateLft == tc->UNDER_GRASPING && stateLft_old == tc->NOT_GRASPING)
		//	seqs.push_back(1);
		//else 
		//	seqs.push_back(2);
		
		for (int j=0;j<tc->bodyItemRobot()->body()->numJoints();j++) {
			seqs.push_back(tc->graspMotionSeq[i].jointSeq[j]*180.0/M_PI); // radian to degree
		}
		//stateLft_old = gstateLft;
	}
	return seqs;	
}
}

namespace cnoid {
void exportPlanning(py::module m) {

	m.doc() = "pybind11 motion planning";
	m.def("add", &add);
	m.def("plan", &Plan_motion);
	m.def("load_motionfile", &Load_motion_file, py::arg("motionfilename"), py::arg("dual_arm")=false);
	m.def("get_motion", &Get_motion);
}
}
