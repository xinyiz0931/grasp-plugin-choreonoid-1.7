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
bool Load_motion_file(string motionfilename) {
	
	FILE *fp;
	if((fp = fopen(motionfilename.c_str(), "r")) != NULL ){
			MotionFileControl::instance()->LoadFromMotionFile(motionfilename);
		fclose(fp);
	}
	TrajectoryPlanner tp;
	bool success = tp.doTrajectoryPlanning();
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

bool Plan_regrasp(vector<double> inixyz, vector<double> inirpy, vector<double> termxyz, vector<double> termrpy, bool init=false)
{
	int success;
	BinPickingControl * bp = BinPickingControl::instance();
	if (init) bp->initJointSeq();

	Vector3 iniPos, iniRPY, termPos, termRPY;
	for (int i=0; i<3; i++) {
		iniPos(i) = inixyz[i];
		iniRPY(i) = inirpy[i];
		termPos(i) = termxyz[i];
		termRPY(i) = termrpy[i];
	}	
	bp->calcJointSeqRegrasp(iniPos, iniRPY, termPos, termRPY);
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
bool Plan_pick(string &arm, vector<double> xyz, vector<double> rpy, bool init=true)
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

	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	if (init) bp->initJointSeq();
	bp->calcJointSeqPick(LR_flag, pickPos, pickRPY);
	success = bp->trajectoryPlanning();
	return success;
}

bool Plan_put(string &arm, vector<double> xyz, vector<double> rpy)
{
	int LR_flag;
	if (arm=="left") LR_flag = 1;
	else LR_flag = 0;
	Vector3 iniPos, iniRPY;
	for (int i=0; i<3; i++) {
		iniPos(i) = xyz[i];
		iniRPY(i) = rpy[i];
	}	
	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	bp->calcJointSeqPut(LR_flag, iniPos, iniRPY);
	success = bp->trajectoryPlanning();
	return success;
}

bool Plan_lift(string &arm, vector<double> sxyz, vector<double> srpy, vector<double> exyz, vector<double> erpy, bool init=false)
{
	Vector3 startPos, startRPY, endPos, endRPY;
	for (int i=0; i<3; i++) {
		startPos(i) = sxyz[i];
		startRPY(i) = srpy[i];
		endPos(i) = exyz[i];
		endRPY(i) = erpy[i];
	}

	int LR_flag;
	if (arm=="left") LR_flag = 1;
	else LR_flag = 0;

	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	if (init) bp->initJointSeq();

	bp->calcJointSeqTwoPoses(LR_flag, startPos, startRPY, endPos, endRPY);
	// bp->calcJointSeqMoveTo(LR_flag, )
	success = bp->trajectoryPlanning();
	
	return success;
}
// bool Plan_fling(string &arm, vector<double> xyz, vector<double> rpy, double j3, double j4, double vel)
bool Plan_swing(string &arm, vector<double> xyz, vector<double> rpy, vector<double> endxyz, vector<double> jrpy, double tm, int repeat=1, bool bilateral=false)
{
	// j = {j3, j4, j5};
	Vector3 startPos, startRPY, eefRPY;
	for (int i=0; i<3; i++) {
		startPos(i) = xyz[i];
		startRPY(i) = rpy[i];
		eefRPY(i) = jrpy[i];
	}
	
	Vector3d endPos;
	if (endxyz.size() > 0)  {
		for (int i=0; i<3; i++) {
			endPos(i) = endxyz[i];
		}
	}

	int LR_flag = 0;
	if (arm == "left") LR_flag = 1;

	int success;
	BinPickingControl * bp = BinPickingControl::instance();

	bp->calcJointSeqSwing(LR_flag, startPos, startRPY, eefRPY, tm, repeat, bilateral, endPos);
	success = bp->trajectoryPlanning();
	return success;
}
bool Plan_spin(string &arm, double vel)
{
	int LR_flag = 0;
	if (arm == "left") LR_flag = 1;
	
	int success;
	BinPickingControl * bp = BinPickingControl::instance();
	bp->calcJointSeqSpin(LR_flag, vel);
	success = bp->trajectoryPlanning();
	return success;
}

bool Plan_transport(string &arm, vector<double> pos, vector<double> rpy)
{
	int LR_flag = 0;
	if (arm == "left") LR_flag = 1;
	// startPos neede? 
	Vector3 startPos = Vector3(0.480, -0.010, 0.480);
	Vector3 startRPY = Vector3(90, -90, -90);
	// Vector3 endPos = Vector3(0.350, -0.350, 0.500);
	// Vector3 endPos = Vector3(0.070, -0.552, 0.500);
	Vector3 endPos = Vector3(-0.050, -0.500, 0.500);
	Vector3 endRPY = startRPY;
	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	bp->calcJointSeqTransport(LR_flag, endPos, endRPY);
	// bp->calcJointSeqTwoPoses(LR_flag, startPos, startRPY, endPos, endRPY);
	success = bp->trajectoryPlanning();
	return success;

}

// bool Plan_move(string &arm, vector<double> xyz, vector<double> rpy)
// {
// 	int LR_flag = 0;
// 	if (arm == "left") LR_flag = 1;
// 	Vector3 endPos, endRPY;
// 	for (int i=0; i<3; i++) {
// 		endPos(i) = xyz[i];
// 		endRPY(i) = rpy[i];
// 	}
// 	int success;
// 	BinPickingControl *bp = BinPickingControl::instance();
// 	bp->calcJointSeqMoveTo(LR_flag, endPos, endRPY);
// 	success = bp->trajectoryPlanning();
// 	return success;
// }

bool Plan_move(string &arm, vector<vector<double>> poses, vector<double> tms)
{
	int LR_Flag = 0;
	if (arm == "left") LR_Flag = 1;

	vector<VectorXd> pos_List;
	VectorXd tmp(6);
	for (int i=0; i<poses.size(); i++) {
		for (int j=0; j<6; j++) {
			tmp(j) = (double)poses[i][j];
		}
		pos_List.push_back(tmp);
	}
	

	int success;
	BinPickingControl *bp = BinPickingControl::instance();
	bp->calcJointSeqMoveTo(LR_Flag, pos_List, tms);
	success = bp->trajectoryPlanning();
	return success;
}
// bool Plan_initialpose()
// bool Plan_moveto(string &arm, vector<double> pos, vector<double> rpy)
// {
// 	int LR_flag = 0;
// 	if (arm == "left") LR_flag = 1;

// 	int success;
// 	BinPickingControl *bp = BinPickingControl::instance();
// 	bp->calcJointSeqMoveTo(LR_flag, endPos)
// }


py::array_t<double> Get_motion() 
{
	int numSeq = PlanBase::instance()->graspMotionSeq.size();
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
	BinPickingControl *bp = BinPickingControl::instance();
	bp->clearJointSeq(valueJnt);
}

vector<double> Get_position(string jnt)
{
	vector<double> pv = {};
	Vector3 pos = PlanBase::instance()->bodyItemRobot()->body()->link(jnt)->p();
	for (int i=0; i<pos.size(); i++)
		pv.push_back(pos(i));
	return pv;
}

namespace cnoid {
void exportPlanning(py::module m) {
	vector<double> emptyv = {};

	m.doc() = "pybind11 motion planning";
	m.def("add", &add);
	m.def("plan", &Plan_motion);
	m.def("load_motionfile", &Load_motion_file, py::arg("motionfilename"));
	m.def("get_motion", &Get_motion);
	m.def("plan_pick", &Plan_pick, py::arg("arm"), py::arg("xyz"), py::arg("rpy"), py::arg("init")=true);
	m.def("plan_lift", &Plan_lift, py::arg("arm"), py::arg("sxyz"), py::arg("srpy"), py::arg("exyz"), py::arg("erpy"), py::arg("init")=false);
	m.def("plan_regrasp", &Plan_regrasp, py::arg("sxyz"), py::arg("srpy"), py::arg("exyz"), py::arg("erpy"),py::arg("init")=false);
	m.def("plan_swing", &Plan_swing, py::arg("arm"), py::arg("xyz"), py::arg("rpy"), py::arg("endxyz"), py::arg("jrpy"), py::arg("tm"), py::arg("repeat")=1, py::arg("bilateral")=false);
	m.def("plan_spin", &Plan_spin, py::arg("arm"), py::arg("vel"));
	// TODO: complete this function!
	m.def("plan_move", &Plan_move, py::arg("arm"), py::arg("poses"), py::arg("tms"));
	m.def("plan_put", &Plan_put, py::arg("arm"), py::arg("xyz"), py::arg("rpy"));

	m.def("plan_binpicking", &Plan_bin_picking, py::arg("arm"), py::arg("pickxyz"), py::arg("pickrpy"), py::arg("placexyz"));
	
	m.def("clear_motion", &Clear_motion, py::arg("jnt")=emptyv);

	m.def("get_position", &Get_position, py::arg("jnt"));
}
}
