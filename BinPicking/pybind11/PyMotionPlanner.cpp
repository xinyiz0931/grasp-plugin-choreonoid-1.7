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
	for (int i; i < size; i++) {
		// one seq = [tm, px19]
		seqs.push_back(tc->graspMotionSeq[i].motionTime);
		
		for (int j=0;j<tc->bodyItemRobot()->body()->numJoints();j++) {
			seqs.push_back(tc->graspMotionSeq[i].jointSeq[j]*180.0/M_PI); // radian to degree
		}
	}
	
	return seqs;	
}
// vector<double> Solve_IK(vector<double> Values, int LR_flag) {
// 	/* ここで、arm変数の引数は右の場合は0で左の場合は1 */
// 	vector<double> seqs;
// 	MotionState tmpState;

// 	int jntNum = PlanBase::instance()->arm(LR_flag)->arm_path->numJoints();
// 	int seqNum = int(Values.size()/6);
// 	for (int i; i < seqNum; i++) {
// 		Vector3 pos = Vector3(Values[i*6+0], Values[i*6+1], Values[i*6+2]);
// 		Vector3 rpy = Vector3(Values[i*6+3], Values[i*6+4], Values[i*6+5]);
// 		rpy = RadConv(rpy);
// 		//手先位置を関節角に変換
// 		PlanBase::instance()->arm(LR_flag)->IK_arm(pos, PlanBase::instance()->arm(LR_flag)->arm_path->joint(jntNum-1)->calcRfromAttitude(rotFromRpy(rpy)));

// 		int jntNum2 = PlanBase::instance()->bodyItemRobot()->body()->numJoints();
// 		VectorXd Value2(jntNum);

// 		if (PlanBase::instance()->graspMotionSeq.size() > 0) 
// 			Value2 = PlanBase::instance()->graspMotionSeq.back().jointSeq;
// 		else {
// 			for (int j=0; j < jntNum2; j++) 
// 				Values2(i) = PlanBase::instance()->bodyItemRobot()->body()->joint(j)->q();
// 		} 

// 		for (int j=0; j<jntNum;j++)
// 			Value2(PlanBase::instance()->arm(LR_flag)->arm_path->joint(j)->jointId()) = PlanBase::instance()->arm(LR_flag)->arm_path->joint(j)->q();
		
// 		tmpState.pos = MotionFileControl::instance()->InitBasePos;
// 		tmpState.rpy = MotionFileControl::instance()->InitBaseRPY;
		
// 	}
// 	MotionFileControl *instance = new MotionFileControl();
// 	FILE *fp;
// 	if ((fp=fopen((motionfilename))))
// }
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
