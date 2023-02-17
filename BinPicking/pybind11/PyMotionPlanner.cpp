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
#include "../../BinPicking/BinPickingControl.h"
#include <pybind11/stl.h> // vector
#include <pybind11/operators.h>//operator
// #include<pybind11/pybind11.h>
#include<pybind11/numpy.h>
using namespace std;
using namespace cnoid;
using namespace grasp;

#define RIGHT_FLAG		0								// 右腕フラグ
#define LEFT_FLAG			1								// 左腕フラグ
#define RAD_CONV			(3.141592/180.0)				// radian変換
#define RadConv(x)		((x)*(3.141592)/(180.0))
#define DEG_CONV			(180.0/3.141592)				// degree変換
#define DegConv(x)		((x)*(180.0)/(3.141592))

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
	
	//MotionFileControl* instance = new MotionFileControl();
	FILE *fp;
	if( (fp = fopen((motionfilename).c_str(), "r")) != NULL ){
			MotionFileControl::instance()->LoadFromMotionFile(motionfilename, dual_arm);
		fclose(fp);
	}
	vector<bool> success = MotionFileControl::instance()->isMotionFileSucceed;
	return success;
}
// vector<double> Get_motion() {
// 	vector<double> seqs;
// 	PlanBase * tc = PlanBase::instance();
// 	int size = tc->graspMotionSeq.size();
// 	for (int i; i < size; i++) {
// 		// one seq = [tm, px19]
// 		// with open/close: one seq = [tm, px22]
// 		seqs.push_back(tc->graspMotionSeq[i].motionTime);
// 		seqs.push_back(tc->graspMotionSeq[i].graspingState);
// 		seqs.push_back(tc->graspMotionSeq[i].graspingState2);
		
// 		for (int j=0;j<tc->bodyItemRobot()->body()->numJoints();j++) {
// 			seqs.push_back(tc->graspMotionSeq[i].jointSeq[j]*180.0/M_PI); // radian to degree
// 			// seqs.push_back(tc->graspMotionSeq[i].jointSeq[j]); 
// 		}
// 	}
// 	cout << endl;
	
// 	return seqs;	
// }

// request 2d array for motion seq

bool Plan_regrasp(bool init=false, bool clear=true)
{
	cout << "[DEBUG] Regrasp, need init? " << init << ", need clear? " << clear << endl;
	int success;
	BinPickingControl * bp = BinPickingControl::instance();
	if (init) bp->initJointSeq();
	// if (clear) bp->clearJointSeq();
	
	bp->calcJointSeqRegrasp();
	success = bp->trajectoryPlanning();
	return success;
}
bool Plan_bin_picking(string &arm, vector<double> pickxyz, vector<double> pickrpy, vector<double> placexyz)
{
	Vector3 pickPos, pickRPY, placePos;
	for (int i=0; i<3; i++) {
		pickPos(i) = pickxyz[i];
		pickRPY(i) = pickrpy[i];
		placePos(i) = placexyz[i];
	}
	int LR_flag;
	// 0->right, 1->left
	if (arm=="left") LR_flag = 1;
	else LR_flag = 0;
	int success;
	BinPickingControl * bp = BinPickingControl::instance();
	bp->initJointSeq();
	bp->doBinPickingPlanning(LR_flag, pickPos, pickRPY, placePos);
	success = bp->trajectoryPlanning();
	return success;

}
bool Plan_pick(string &arm, vector<double> xyz, vector<double> rpy, bool init=true, bool clear=true)
{
	Vector3 pickPos, pickRPY;
	for (int i=0; i<3; i++) {
		pickPos(i) = xyz[i];
		pickRPY(i) = rpy[i];
	}

	int LR_flag;
	// 0->right, 1->left
	if (arm=="left") LR_flag = 1;
	else LR_flag = 0;
	cout << "[DEBUG] Pick, need init? " << init << ", need clear? " << clear << endl;

	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	if (init) bp->initJointSeq();
	// if (clear) bp->clearJointSeq();
	bp->calcJointSeqPick(LR_flag, pickPos, pickRPY);
	success = bp->trajectoryPlanning();
	return success;
}

bool Plan_lift(string &arm, vector<double> sxyz, vector<double> srpy, vector<double> exyz, bool init=false, bool clear=true)
{
	Vector3 startPos, startRPY, endPos;
	for (int i=0; i<3; i++) {
		startPos(i) = sxyz[i];
		startRPY(i) = srpy[i];
		endPos(i) = exyz[i];
	}

	int LR_flag;
	if (arm=="left") LR_flag = 1;
	else LR_flag = 0;
	cout << "[DEBUG] Pick, need init? " << init << ", need clear? " << clear << endl;

	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	if (init) bp->initJointSeq();
	// if (clear) bp->clearJointSeq();

	bp->calcJointSeqTwoPoses(LR_flag, startPos, startRPY, endPos);
	success = bp->trajectoryPlanning();
	return success;
}

bool Plan_fling()
{
	Vector3 pos = Vector3(0.480, -0.010, 0.480);
	Vector3 rpy = Vector3(90, -90, -90);

	int LR_flag = 0;
	int success;
	BinPickingControl * bp = BinPickingControl::instance();
	bp->calcJointSeqFling(LR_flag, pos, rpy, 30.0, 60.0);
	success = bp->trajectoryPlanning();
	return success;
}

bool Plan_place()
{
	Vector3 startPos = Vector3(0.480, -0.010, 0.480);
	Vector3 startRPY = Vector3(90, -90, -90);
	// Vector3 endPos = Vector3(0.350, -0.350, 0.500);
	Vector3 endPos = Vector3(0.070, -0.552, 0.500);
	int LR_flag = 0;
	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	bp->calcJointSeqTwoPoses(LR_flag, startPos, startRPY, endPos);
	success = bp->trajectoryPlanning();
	return success;

}


py::array_t<double> Get_motion() 
{
	int numSeq = PlanBase::instance()->graspMotionSeq.size();
	cout << "[DEBUG] Grasp sequence size: " << numSeq << endl;
	int size = 3 + PlanBase::instance()->bodyItemRobot()->body()->numJoints(); // tm + rhand + lhand + jnt(19)
    auto result = py::array_t<double>({numSeq,size});

    py::buffer_info buf_result = result.request();

    double* ptr_result = (double*)buf_result.ptr;
	
	// cout << "[DEBUG] Grasp sequence " << endl;
    for (int i=0; i<numSeq; i++)
    {
		// cout << "#" <<i<<": "<< "("<<PlanBase::instance()->graspMotionSeq[i].motionTime<<")"<<"(" <<PlanBase::instance()->graspMotionSeq[i].graspingState<<PlanBase::instance()->graspMotionSeq[i].graspingState2 << ") ";
		ptr_result[i*buf_result.shape[1]+0] = PlanBase::instance()->graspMotionSeq[i].motionTime;
		ptr_result[i*buf_result.shape[1]+1] = PlanBase::instance()->graspMotionSeq[i].graspingState;
		ptr_result[i*buf_result.shape[1]+2] = PlanBase::instance()->graspMotionSeq[i].graspingState2;
        for (int j=0; j<PlanBase::instance()->bodyItemRobot()->body()->numJoints(); j++)
        {
            ptr_result[i*buf_result.shape[1]+j+3] = PlanBase::instance()->graspMotionSeq[i].jointSeq[j]*180/M_PI;
			// cout <<PlanBase::instance()->graspMotionSeq[i].jointSeq[j] << ' ';
        }
		// cout << '\n';
    }
	// cout << "-----------------------------------------------" << endl;
    return result;
}
}

void Clear_motion(vector<double> jnt)
{
	VectorXd valueJnt;
	valueJnt.resize(jnt.size());
	if (jnt.size() > 1) {
		for (int i=0; i<jnt.size(); i++)
			valueJnt(i) = jnt[i];
	}
	cout << "Clear_motion() : " << jnt.size()  << endl;
	BinPickingControl *bp = BinPickingControl::instance();
	bp->clearJointSeq(valueJnt);
}

namespace cnoid {
void exportPlanning(py::module m) {

	m.doc() = "pybind11 motion planning";
	m.def("add", &add);
	m.def("plan", &Plan_motion);
	m.def("load_motionfile", &Load_motion_file, py::arg("motionfilename"), py::arg("dual_arm")=false);
	m.def("get_motion", &Get_motion);
	m.def("plan_pick", &Plan_pick, py::arg("arm"), py::arg("xyz"), py::arg("rpy"), py::arg("init")=true, py::arg("clear")=false);
	m.def("plan_lift", &Plan_lift, py::arg("arm"), py::arg("sxyz"), py::arg("srpy"), py::arg("exyz"), py::arg("init")=false, py::arg("clear")=true);
	m.def("plan_regrasp", &Plan_regrasp, py::arg("init")=false, py::arg("clear")=true);
	// m.def("plan_fling", &Plan_fling, py::arg("arm"), py::arg("xyz"), py::arg("rpy"), py::arg("j3"), py::arg("j4));
	m.def("plan_fling", &Plan_fling);
	m.def("plan_place", &Plan_place);

	m.def("plan_binpicking", &Plan_bin_picking, py::arg("arm"), py::arg("pickxyz"), py::arg("pickrpy"), py::arg("placexyz"));
	m.def("clear_motion", &Clear_motion, py::arg("jnt"));
}
}
