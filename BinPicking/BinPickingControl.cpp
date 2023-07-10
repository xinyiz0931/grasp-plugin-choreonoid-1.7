/**
   @author Takashi Kitagawa (AIST)
   copied by xinyi 2022/12/1
*/

#include "BinPickingControl.h"
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <cnoid/ExecutablePath>
#include <cnoid/MessageView>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <iostream>
#include <fstream>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/ViewManager>
#endif

#define RIGHT_FLAG		0								// 右腕フラグ
#define LEFT_FLAG			1								// 左腕フラグ
#define RAD_CONV			(3.141592/180.0)				// radian変換
#define RadConv(x)		((x)*(3.141592)/(180.0))
#define DEG_CONV			(180.0/3.141592)				// degree変換
#define DegConv(x)		((x)*(180.0)/(3.141592))
// #define DEBUG_MODE 0

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

BinPickingControl* BinPickingControl::instance()
{
    static BinPickingControl* instance = new BinPickingControl();
    return instance;
}

BinPickingControl::BinPickingControl()
	: mes(*MessageView::mainInstance()),
	os (MessageView::mainInstance()->cout() )
{
	init		= 0;
	initRegrasp = 0;
	gc = PlanBase::instance();
	numJnt = gc->bodyItemRobot()->body()->numJoints();

	
	tmpState.graspingState = PlanBase::NOT_GRASPING;
	tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	InitBasePos = Vector3( 0.0, 0.0, 0.0 );
	InitBaseRPY = Vector3( 0.0, 0.0, 0.0 );
	_InitJnt = {0,0,0,-0.174533,-0.44855,-2.22529,0,0,0,0.401426,-0.44855,-2.22529,-0.122173,0,0,0.026,-0.026,0.026,-0.026};
	// following joint info: for openlab demo
	// _InitJnt = {0,0,0,-0.232334,-0.132166,-2.16145,-0.343502,-0.818023,0.517139,0.182075,-0.474497,-2.32239,0.193182,-0.338776,-0.326721,0,0,0,0};
	
	InitJnt.resize(_InitJnt.size());
	for (int i=0; i<numJnt; i++) {
		InitJnt(i) = _InitJnt[i];
	}
	

	
}

BinPickingControl::~BinPickingControl()
{

}

void BinPickingControl::SaveJnt(string savepath)
{
    cout << "[*] Saved current pose in " << savepath << endl;
	PlanBase *tc = tc;

	ofstream gout(savepath);
	for (int i=0; i<gc->bodyItemRobot()->body()->numJoints(); i++) {
		// gout << DegConv(gc->bodyItemRobot()->body()->joint(i)->q()) << " ";
		gout << gc->bodyItemRobot()->body()->joint(i)->q() << ",";
	}
	gout << endl;
	for (int i=0; i<gc->bodyItemRobot()->body()->numJoints(); i++) {
		gout << DegConv(gc->bodyItemRobot()->body()->joint(i)->q()) << " ";
	}
	gout << endl;
}

void BinPickingControl::calcArmJointSeq(VectorXd& armJointSeq, ArmPtr& arm) 
{
	for (int i=0; i< arm->arm_path->numJoints(); i++) {
		armJointSeq(i) = gc->graspMotionSeq.back().jointSeq(arm->arm_path->joint(i)->jointId());
		cout << DegConv(armJointSeq(i)) << " ";
	}
}
// void BinPickingControl::calcJointSeq(VectorXd& valueJnt) 
// {
// 	if (gc->graspMotionSeq.size() > 0) {
// 		valueJnt = gc->graspMotionSeq.back().jointSeq;
// 	}
// 	else {
// 		for (int i=0; i<numJnt; i++) {
// 			valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
// 		}
// 	}
// }

void BinPickingControl::setHandStateSeq(int LR_flag, int handState) 
{
	int numJnt = gc->bodyItemRobot()->body()->numJoints();
	VectorXd valueJnt(numJnt);
	double T= 0.5;
	if (gc->graspMotionSeq.size() > 0) {
		valueJnt = gc->graspMotionSeq.back().jointSeq;
	}
	else {
		for (int i=0; i<numJnt; i++) {
			valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		}
	}


	for (int i=0; i<gc->nFing(LR_flag); i++) {
		if (handState == HAND_CLOSING) {
			// close: 4
			valueJnt(gc->fingers(LR_flag,i)->fing_path->joint(0)->jointId()) = 0;
		}
		else if (handState == HAND_OPENING) {
			// open: 3
			valueJnt(gc->fingers(LR_flag,i)->fing_path->joint(0)->jointId()) = gc->fingers(LR_flag,i)->fingerOpenPose[0];
		}
	}
	
	if (LR_flag==0) {
		tmpState.graspingState = handState;
		tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	}
	else {
		tmpState.graspingState = PlanBase:: NOT_GRASPING;
		tmpState.graspingState2 = handState;
	}

	tmpState.pos = InitBasePos;
	tmpState.rpy = InitBaseRPY;
	tmpState.jointSeq = valueJnt;
	tmpState.motionTime = T;

	tmpState.pathPlanDOF.clear();
	gc->graspMotionSeq.push_back(tmpState);
	// gc->bodyItemRobot()->body()->calcForwardKinematics();
	// gc->bodyItemRobot()->notifyKinematicStateChange();

}

void BinPickingControl::calcIK(int LR_flag, Vector3d pos, Vector3d rpy, VectorXd &jnt)
{
	int numArmJnt = gc->arm(LR_flag)->arm_path->numJoints();
	Matrix3 rot = gc->arm(LR_flag)->arm_path->joint(numArmJnt-1)->calcRfromAttitude(rotFromRpy(rpy));
	
	gc->arm(LR_flag)->IK_arm(pos, rot);

	jnt(numJnt);
	if (gc->graspMotionSeq.size() > 0) {
		jnt = gc->graspMotionSeq.back().jointSeq;
	}
	else {
		for (int i=0; i<numJnt; i++) {
			jnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		}
	}

	for (int i=0; i<numArmJnt; i++) {
		jnt(gc->arm(LR_flag)->arm_path->joint(i)->jointId()) = gc->arm(LR_flag)->arm_path->joint(i)->q();
	}
}

void BinPickingControl::setMotionSeq(int LR_flag, double T, Vector3d pos, Vector3d rpy) 
{
	int numJnt = gc->bodyItemRobot()->body()->numJoints();
	int numArmJnt = gc->arm(LR_flag)->arm_path->numJoints();

	Matrix3 rot = gc->arm(LR_flag)->arm_path->joint(numArmJnt-1)->calcRfromAttitude(rotFromRpy(rpy));
	
	gc->arm(LR_flag)->IK_arm(pos, rot);

	VectorXd valueJnt(numJnt);
	if (gc->graspMotionSeq.size() > 0) {
		valueJnt = gc->graspMotionSeq.back().jointSeq;
	}
	else {
		for (int i=0; i<numJnt; i++) {
			valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		}
	}

	for (int i=0; i<numArmJnt; i++) {
		valueJnt(gc->arm(LR_flag)->arm_path->joint(i)->jointId()) = gc->arm(LR_flag)->arm_path->joint(i)->q();
	}
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->bodyItemRobot()->notifyKinematicStateChange();

	tmpState.pos = InitBasePos;
	tmpState.rpy = InitBaseRPY;
	tmpState.jointSeq = valueJnt;
	tmpState.motionTime = T;
	tmpState.graspingState = PlanBase::NOT_GRASPING;
	tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	tmpState.pathPlanDOF.clear();
	for (int i=0; i<numArmJnt; i++) {
		tmpState.pathPlanDOF.push_back(gc->arm(LR_flag)->arm_path->joint(i)->jointId());
	}
	gc->graspMotionSeq.push_back(tmpState);
}

void BinPickingControl::setMotionSeqDual(int LR_flag, double T, Vector3d pos, Vector3d rpy, bool is_fixwaist) 
{
	int numJnt = gc->bodyItemRobot()->body()->numJoints();
	int numArmJnt = gc->arm(LR_flag)->arm_path->numJoints();

	// arm: main arm to plan motion
	// arm_: support arm needs to be static
	arm = gc->arm(LR_flag);
	arm_ = gc->arm(1-LR_flag);	

	Matrix3 rot = arm->arm_path->joint(numArmJnt-1)->calcRfromAttitude(rotFromRpy(rpy));

	Vector3 pos2 = arm_->palm->p();
	Vector3 rpy2 = rpyFromRot(arm_->palm->attitude());
	Matrix3 rot2 = arm_->arm_path->joint(numArmJnt-1)->calcRfromAttitude(rotFromRpy(rpy2));

	if (is_fixwaist) {
		arm->IK_arm(pos, rot, arm->arm_path->joint(0)->q());
		arm_->IK_arm(pos2, rot2, arm_->arm_path->joint(0)->q());
	}
	else {
		arm->IK_arm(pos, rot);
		arm_->IK_arm(pos2, rot2);
	}

	VectorXd valueJnt(numJnt);

	if (gc->graspMotionSeq.size() > 0) {
		valueJnt = gc->graspMotionSeq.back().jointSeq;
	}
	else {
		for (int i=0; i<numJnt; i++) {
			valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		}
	}

	for (int i=0; i<numArmJnt; i++) {
		valueJnt(arm->arm_path->joint(i)->jointId()) = arm->arm_path->joint(i)->q();
		valueJnt(arm_->arm_path->joint(i)->jointId()) = arm_->arm_path->joint(i)->q();
	}
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->bodyItemRobot()->notifyKinematicStateChange();

	tmpState.pos = InitBasePos;
	tmpState.rpy = InitBaseRPY;
	tmpState.jointSeq = valueJnt;
	tmpState.motionTime = T;
	tmpState.graspingState = PlanBase::NOT_GRASPING;
	tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	tmpState.pathPlanDOF.clear();
	
	for (int i=0; i<numJnt; i++) {
		tmpState.pathPlanDOF.push_back(i);
	}

	gc->graspMotionSeq.push_back(tmpState);

}

void BinPickingControl::setJointSeq(double T, VectorXd jnt)
{
	tmpState.pos = InitBasePos;
	tmpState.rpy = InitBaseRPY;
	tmpState.jointSeq = jnt;
	tmpState.motionTime = T;
	tmpState.graspingState = PlanBase::NOT_GRASPING;
	tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	tmpState.pathPlanDOF.clear();
	for (int i=0; i<numJnt; i++) {
		tmpState.pathPlanDOF.push_back(i);
	}

	gc->graspMotionSeq.push_back(tmpState);
	
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->bodyItemRobot()->notifyKinematicStateChange();

}
void BinPickingControl::setArmJointSeq(int LR_flag, double T, VectorXd armJnt)
{
	int numJnt = gc->bodyItemRobot()->body()->numJoints();	
	int numArmJnt = gc->arm(LR_flag)->arm_path->numJoints();
	VectorXd valueJnt(numJnt);
	for (int i=0; i<numJnt; i++) {
		// valueJnt(i) = gc->graspMotionSeq.back().jointSeq(i);
		valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
	}
	
	for (int i=0; i<numArmJnt; i++) { 
		if (i != 0) { // do not revise waist joint!!! 
			valueJnt(gc->arm(LR_flag)->arm_path->joint(i)->jointId()) = armJnt(i);
		}
	}

	tmpState.pos = InitBasePos;
	tmpState.rpy = InitBaseRPY;
	tmpState.jointSeq = valueJnt;
	tmpState.motionTime = T;
	tmpState.graspingState = PlanBase::NOT_GRASPING;
	tmpState.graspingState2 = PlanBase::NOT_GRASPING;
	tmpState.pathPlanDOF.clear();
	for (int i=0; i<numArmJnt; i++) 
		tmpState.pathPlanDOF.push_back(gc->arm(LR_flag)->arm_path->joint(i)->jointId());
	gc->graspMotionSeq.push_back(tmpState);
	
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->bodyItemRobot()->notifyKinematicStateChange();

}

void BinPickingControl::calcJointSeqRegrasp(Vector3d initGraspPos, Vector3d initGraspRPY, Vector3d terminatePos, Vector3d terminateRPY)
{
	int repeat = 1;

	int LR_flag = RIGHT_FLAG; // main arm equipped with force sensor
	
	double regraspOffset = 0.03;


	// Vector3 initGraspPos = Vector3(0.480, -0.170, 0.450);
	// Vector3 regraspPos = Vector3(initGraspPos(0), 0.130, initGraspPos(2)-regraspOffset);
	Vector3 regraspPos = Vector3(initGraspPos(0), initGraspPos(1)+0.3, initGraspPos(2)-regraspOffset);

	vector<Vector3> regraspRPY;
	// regraspRPY.push_back(RadConv(Vector3(180.0, 0.0, -90.0))); // right arm
	// regraspRPY.push_back(RadConv(Vector3(0.0, 0.0, -90.0))); // right arm
	regraspRPY.push_back(RadConv(initGraspRPY)); // right arm
	regraspRPY.push_back(RadConv(Vector3(180.0, 0.0, 90.0))); // left arm
	
	Vector3 tmpPos;

	setMotionSeq(LR_flag, 1.0, initGraspPos, regraspRPY[LR_flag]);

	double backOffset = 0.08;
	double upOffset = 0.11;
	double downOffset = 0.15;
	
	// define keyposes
	for (int i=0; i<repeat; i++) {

		// **support**: back pose
		tmpPos = Vector3(regraspPos(0), regraspPos(1)+backOffset, regraspPos(2));		
		setMotionSeqDual(1-LR_flag, 1.0, tmpPos, regraspRPY[1-LR_flag]);
		
		// **support**: approach
		setMotionSeqDual(1-LR_flag, 1.0, regraspPos, regraspRPY[1-LR_flag]);

		// **support**: pushing to locate cable
		tmpPos = Vector3(regraspPos(0), regraspPos(1)-0.1, regraspPos(2)-downOffset);		
		setMotionSeqDual(1-LR_flag, 3.0, tmpPos, regraspRPY[1-LR_flag]);

		// **support**: close, **main**: open
		setHandStateSeq(1-LR_flag, HAND_CLOSING);
		setHandStateSeq(LR_flag, HAND_OPENING);

		// **main**: back pose		
		tmpPos = Vector3(initGraspPos(0), initGraspPos(1)-backOffset, initGraspPos(2));
		setMotionSeqDual(LR_flag, 1.0, tmpPos, regraspRPY[LR_flag]);

		// **support**: moving up
		tmpPos = Vector3(regraspPos(0), regraspPos(1), initGraspPos(2)+regraspOffset);		
		setMotionSeqDual(1-LR_flag, 1.0, tmpPos, regraspRPY[1-LR_flag]);

#ifdef DEBUG_MODE
		cout << "[DEBUG] Finger 0 position: " << gc->fingers(1-LR_flag,0)->tip->p().transpose() << endl;
		cout << "[DEBUG] Finger 1 position: " << gc->fingers(1-LR_flag,1)->tip->p().transpose() << endl;
		cout << "[DEBUG] Wrist  position: " << gc->arm(1-LR_flag)->palm->p().transpose() << endl;;
#endif
		Vector3 tracePos = (gc->fingers(1-LR_flag,0)->tip->p() + gc->fingers(1-LR_flag,1)->tip->p())/2 + Vector3(0,0,-regraspOffset);
		Vector3 fingPos = gc->fingers(1-LR_flag,0)->tip->p();
		Matrix3 fingRPY = gc->fingers(1-LR_flag,0)->tip->attitude();
		tmpPos = tracePos + fingRPY * Vector3(-0.090-0.0882, 0, 0); // finger length

		// **main**: approach
		setMotionSeqDual(LR_flag, 1.0, tmpPos, regraspRPY[LR_flag]);
		
		// **main**: locate cable 
		setMotionSeqDual(LR_flag, 1.0, Vector3(tmpPos(0),tmpPos(1)+0.03, tmpPos(2)-0.07), regraspRPY[LR_flag]);

		// **main**: close, **support**: open
		setHandStateSeq(LR_flag, HAND_CLOSING);
		setHandStateSeq(1-LR_flag, HAND_OPENING);
		
		// **support**: back pose
		// tmpPos = Vector3(regraspPos(0), regraspPos(1)+backOffset, initGraspPos(2)-regraspOffset);		
		tmpPos = Vector3(regraspPos(0), regraspPos(1)+downOffset, initGraspPos(2)-regraspOffset);		
		setMotionSeqDual(1-LR_flag, 2.0, tmpPos, regraspRPY[1-LR_flag]);
	}

	// **main**: move the the ending pose
	setMotionSeq(LR_flag, 2.0, terminatePos, RadConv(terminateRPY));
	
	// **support**: back to initial pose
	setArmJointSeq(1-LR_flag, 2.0, InitArmJnt[1-LR_flag]);

	
}
void BinPickingControl::calcJointSeqPut(int LR_flag, Vector3 pos, Vector3 rpy)
{
	setMotionSeq(LR_flag, 1.0, pos, RadConv(rpy));
	setHandStateSeq(LR_flag, HAND_OPENING);
	setJointSeq(2.0, InitJnt);
}

void BinPickingControl::calcJointSeqPick(int LR_flag, Vector3 pos, Vector3 rpy)
{
	double bufferH = 0.30;
	// setMotionSeq(LR_flag, 1.2, Vector3(pickP(0),pickP(1),bufferH), RadConv(pickR));
	// setMotionSeq(LR_flag, 0.8, Vector3(pickP(0),pickP(1),pickP(2)+0.02), RadConv(pickR));
	// setMotionSeq(LR_flag, 0.8, pickP, RadConv(pickR));
	
	// setHandStateSeq(LR_flag, HAND_CLOSING);

	setMotionSeq(LR_flag, 1.5, Vector3(pos(0),pos(1),bufferH), RadConv(rpy));
	setHandStateSeq(LR_flag, HAND_CLOSING);
	setHandStateSeq(LR_flag, HAND_OPENING);
	setMotionSeq(LR_flag, 1.0, Vector3(pos(0),pos(1),pos(2)+0.02), RadConv(rpy));
	setMotionSeq(LR_flag, 1.0, pos, RadConv(rpy));
	setHandStateSeq(LR_flag, HAND_CLOSING);
}

void BinPickingControl::calcJointSeqTransport(int LR_flag, Vector3 pos, Vector3 rpy)
{
	setMotionSeq(LR_flag, 6.0, pos, RadConv(rpy));
	// setMotionSeq(LR_flag, 2.0, Vector3(pos(0),pos(1),pos(2)-0.1), RadConv(rpy));
	// setHandStateSeq(LR_flag, HAND_OPENING);

	// setJointSeq(3.0, InitJnt);
}

void BinPickingControl::calcJointSeqTwoPoses(int LR_flag, Vector3 startpos, Vector3 startrpy, Vector3 endpos, Vector3 endrpy)
{

	setMotionSeq(LR_flag, 3.0, startpos, RadConv(startrpy));
	setMotionSeq(LR_flag, 6.0, endpos, RadConv(endrpy));
}

void BinPickingControl::calcJointSeqMoveTo(int LR_flag, vector<VectorXd> poses, vector<double> tms)
{
	Vector3d xyz, rpy;
	for (int i; i<poses.size(); i++) {
		for (int j=0; j<3; j++) {
			xyz(j) = poses[i][j];
			rpy(j) = poses[i][3+j];
		}
		setMotionSeq(LR_flag, tms[i], xyz, RadConv(rpy));
	}
}

void BinPickingControl::initJointSeq()
{
	gc->graspMotionSeq.clear();
	motionSeq.clear();
	
	int numJnt = gc->bodyItemRobot()->body()->numJoints();

	InitJnt.resize(numJnt);
	for (int i=0; i<numJnt; i++) {
		// InitJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		InitJnt(i) = _InitJnt[i];
	}
#ifdef DEBUG_MODE
	cout << "[DEBUG] Initial arm pose" << endl;

#endif
	for (int i=0; i<gc->armsList.size(); i++) {
		// when init: oen all gripper
		for (int j=0; j<gc->nFing(i); j++) {
			InitJnt(gc->fingers(i,j)->fing_path->joint(0)->jointId()) = gc->fingers(i,j)->fingerOpenPose[0];
		}
		// when init: record arm joint for initial pose without waist joint
		VectorXd valueArmJnt(gc->arm(i)->arm_path->numJoints());
		for (int j=0; j<gc->arm(i)->arm_path->numJoints(); j++) {
			valueArmJnt(j) = InitJnt(gc->arm(i)->arm_path->joint(j)->jointId());	
		}
		InitArmJnt.push_back(valueArmJnt);
#ifdef DEBUG_MODE
		cout << "        ";
		for (int j=0; j<gc->arm(i)->arm_path->numJoints(); j++) {
			valueArmJnt(j) = InitJnt(gc->arm(i)->arm_path->joint(j)->jointId());	
			cout << valueArmJnt(j) << " ";
		}
		cout << endl;
#endif
	}
	// InitBasePos = gc->bodyItemRobot()->body()->link(0)->p();
	// InitBaseRPY = rpyFromRot(gc->bodyItemRobot()->body()->link(0)->attitude());
	
	MotionState tmpState;
	tmpState.pos = InitBasePos;
	tmpState.rpy = InitBaseRPY;
	tmpState.jointSeq = InitJnt;
	tmpState.motionTime = 1.0;
	tmpState.pathPlanDOF.clear();

	// When initialize, open both gripper first! 
	tmpState.graspingState = HAND_OPENING;
	tmpState.graspingState2 = HAND_OPENING;

	for (int j=0; j<numJnt; j++) {
		tmpState.pathPlanDOF.push_back(gc->bodyItemRobot()->body()->joint(j)->jointId());
	}
	gc->graspMotionSeq.push_back(tmpState);
	motionSeq.clear();
}

void BinPickingControl::clearJointSeq(VectorXd jnt)
{
	VectorXd valueJnt, lastJnt;
	valueJnt.resize(numJnt);
	if (jnt.size() < 1) {
		valueJnt = gc->graspMotionSeq.back().jointSeq;
	}
	else if (jnt.size() == numJnt -4) { // 15
		lastJnt = gc->graspMotionSeq.back().jointSeq;
		for (int i=0; i<numJnt; i++) {
			if (i < jnt.size()) 
				valueJnt(i) = jnt(i);
			else
				valueJnt(i) = lastJnt(i);
		}
	}
	else if (jnt.size() == numJnt) {
		valueJnt = jnt;
		cout << "[DEBUG] insert given jnt: "<< endl;
	}
#ifdef DEBUG_MODE
		cout << valueJnt.transpose() << endl;
#endif
	gc->graspMotionSeq.clear();
	
	tmpState.pos = InitBasePos;
	tmpState.rpy = InitBaseRPY;
	tmpState.jointSeq = valueJnt;
	tmpState.motionTime = 1.0;
	tmpState.pathPlanDOF.clear();

	tmpState.graspingState = PlanBase::NOT_GRASPING;
	tmpState.graspingState2 = PlanBase::NOT_GRASPING;

	for (int j=0; j<numJnt; j++) {
		tmpState.pathPlanDOF.push_back(gc->bodyItemRobot()->body()->joint(j)->jointId());
	}
	gc->graspMotionSeq.push_back(tmpState);
	// motionSeq.clear();
}

void BinPickingControl::setRelJoint(double T, string jntName, double jntAngle)
{
	cout << "setRelJoint()" << endl;
	int id = gc->bodyItemRobot()->body()->link(jntName)->jointId();
	
	// setMotionSeq(LR_flag, 3.0, pos, RadConv(rpy));
	VectorXd valueJnt(numJnt);

	for (int i=0; i<numJnt; i++) {
		if (i == id) {
			valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(jntAngle);
		}
		else {
			valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		}
	}
	setJointSeq(T, valueJnt);
}
void BinPickingControl::calcJointSeqSpin(int LR_flag, double velocity)
{
	string n;
	if (LR_flag == 0) n="R";
	else n="L";
	
	int j5 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT5")->jointId();
	int numArmJnt = gc->arm(LR_flag)->arm_path->numJoints();
	
	// setMotionSeq(LR_flag, 3.0, pos, RadConv(rpy));
	VectorXd valueJnt(numJnt);
	VectorXd spinJnt1(numJnt), spinJnt2(numJnt);

	for (int i=0; i<numJnt; i++) {
		valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		spinJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		spinJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		if (i==j5) {
			spinJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(90);
			spinJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q() - RadConv(60);
		}

	}
	if (velocity > 1.0 and velocity < 0.1) 
		velocity = 1.0;
	setJointSeq(velocity, spinJnt1);
	setJointSeq(velocity, spinJnt2);
	
	setJointSeq(velocity, valueJnt);

}
void BinPickingControl::calcJointSeqShake(int LR_Flag, Vector3d startpos, Vector3d startrpy, Vector3d endpos, int n)
{
	cout << "TEST shaking funciton! " << endl;
	double T = 0.3;
	for (int i=0; i<n; i++) {
		Vector3d tmpPos;
		Vector3d tmpRPY1 = startrpy;
		Vector3d tmpRPY2 = startrpy;
		tmpRPY1(1) += 5;
		tmpRPY2(1) -= 5;
		tmpPos = startpos + (endpos-startpos)/n*(i+1);
		setMotionSeq(LR_Flag, T, tmpPos, RadConv(tmpRPY1));
		setMotionSeq(LR_Flag, T, tmpPos, RadConv(tmpRPY2));
	}
}

void BinPickingControl::calcJointSeqSwing(int LR_flag, Vector3d pos, Vector3d rpy, Vector3d angles, double T, int repeat, bool bilateral, Vector3d endPos)
{
	string n;
	if (LR_flag == 0) n="R";
	else n="L";
	// double T = 0.4;
	if (T > 1.0 and T < 0.1) 
		T = 0.5;

	int j3 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT3")->jointId();
	int j4 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT4")->jointId();
	int j5 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT5")->jointId();

	// VectorXd valueJnt(numJnt);
	// valueJnt = gc->graspMotionSeq.back().jointSeq;

	// for (int i=0; i<numJnt; i++) 
	// 	valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
	
	// VectorXd flingJnt(numJnt);
	// flingJnt = valueJnt;
	setMotionSeq(LR_flag, 3, pos, RadConv(rpy));
	VectorXd valueJnt(numJnt), swingJnt(numJnt);

	for (int i=0; i<repeat; i++) {
		Vector3d tmpPos;
		// tmpPos = pos + (endPos-pos)/repeat*(i+1);
		tmpPos = pos + (endPos-pos)*((i+1)*2-1)/(repeat*2);
		VectorXd tmpJnt;
		calcIK(LR_flag, tmpPos, RadConv(rpy), tmpJnt);
		// setMotionSeq(LR_flag, T, tmpPos, RadConv(rpy));

		tmpPos = pos + (endPos-pos)*(i+1)/repeat;
		calcIK(LR_flag, tmpPos, RadConv(rpy), tmpJnt);
		// for (int j=0; j<numJnt; j++) 
		// 	valueJnt(j) = gc->bodyItemRobot()->body()->joint(j)->q();

		// flingJnt = valueJnt;
		swingJnt = tmpJnt;
		swingJnt(j3) = swingJnt(j3) - RadConv(angles(0));
		swingJnt(j4) = swingJnt(j4) - RadConv(angles(1));
		swingJnt(j5) = swingJnt(j5) - RadConv(angles(2));
		setJointSeq(T, swingJnt);
		
		if (bilateral) {
			VectorXd swingJnt2 = tmpJnt;
			swingJnt2(j3) = swingJnt2(j3) + RadConv(angles(0));
			swingJnt2(j4) = swingJnt2(j4) + RadConv(angles(1));
			swingJnt2(j5) = swingJnt2(j5) + RadConv(angles(2));
			setJointSeq(T, swingJnt2);
		}
		else {
			setJointSeq(T, tmpJnt);
		}

		// for (int j=0; j<numJnt; j++) {
		// 	if (j == j3) 
		// 		flingJnt(j) = flingJnt(j) - RadConv(angleJnt3);
		// 	else if (j == j4)
		// 		flingJnt(j) = flingJnt(j) - RadConv(angleJnt4);
		// 	else if (j == j5)
		// 		flingJnt(j) = flingJnt(j) - RadConv(angleJnt5);
		// }
		// cout << "=== " << i << ": " << flingJnt.transpose() << endl; 	

		// setRelJoint(T, n+"ARM_JOINT5", angleJnt5);
		// setJointSeq(T, flingJnt);
		// tmpPos = pos + (endPos-pos)/repeat*(i+1);
		
		// if (!bilateral){
		// 	setMotionSeq(LR_flag, T, tmpPos, RadConv(rpy));
		// }
	}
	// for (int i=0; i<numJnt; i++) {
	// 	if (i == j3) {
	// 		flingJnt(i) = flingJnt(i) - RadConv(angleJnt3);
	// 	}
	// 	else if (i == j4)
	// 	{
	// 		flingJnt(i) = flingJnt(i) - RadConv(angleJnt4);
	// 	}
	// 	else if (i == j5)
	// 	{
	// 		flingJnt(i) = flingJnt(i) - RadConv(angleJnt5);
	// 	}
	// }
	// double T = 1.0;
	// for (int i=0; i<repeat; i++) {
	// 	setJointSeq(T, flingJnt);
	// 	setJointSeq(T, valueJnt);
	// }
}
// void BinPickingControl::calcJointSeqSwing(int LR_flag, Vector3 pos, Vector3 rpy, double angleJnt3, double angleJnt4, double velocity, int repeat) 
// {
// 	angleJnt4 = 45;
// 	angleJnt3 = 0.0;
// 	string n;
// 	if (LR_flag == 0) n="R";
// 	else n="L";
	
// 	int j3 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT3")->jointId();
// 	int j4 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT4")->jointId();
// 	int j5 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT5")->jointId();
// 	int numArmJnt = gc->arm(LR_flag)->arm_path->numJoints();
	
// 	setMotionSeq(LR_flag, 3.0, pos, RadConv(rpy));

// 	VectorXd valueJnt(numJnt);
// 	// valueJnt = gc->graspMotionSeq.back().jointSeq;
// 	for (int i=0; i<numJnt; i++) {
// 		valueJnt(i) = gc->bodyItemRobot()->body()->joint(i)->q();
// 	}

// 	VectorXd flingJnt1(numJnt), flingJnt2(numJnt);
// 	flingJnt1 = valueJnt;
// 	flingJnt2 = valueJnt;

// 	VectorXd spinJnt1(numJnt), spinJnt2(numJnt);
// 	spinJnt1 = valueJnt;
// 	spinJnt2 = valueJnt;
	
// 	for (int i=0; i<numJnt; i++) {

// 		if (i == j3) {
// 			flingJnt1(i) = flingJnt1(i) + RadConv(angleJnt3);
// 			flingJnt2(i) = flingJnt2(i) - RadConv(angleJnt3);
// 		}
// 		else if (i==j4) {
// 			flingJnt1(i) = flingJnt1(i) + RadConv(angleJnt4);
// 			flingJnt2(i) = flingJnt2(i) - RadConv(angleJnt4);
// 		}
// 		else if (i==j5) {
// 			flingJnt2(i) = flingJnt2(i) + RadConv(70);
// 			spinJnt1(i) = spinJnt1(i) + RadConv(110);
// 			spinJnt2(i) = spinJnt2(i) - RadConv(70);
// 		}
// 	}

// 	if (velocity > 1.0 and velocity < 0.1) 
// 		velocity = 1.0;

// 	// setJointSeq(velocity, spinJnt1);
// 	// setJointSeq(velocity, spinJnt2);
// 	for (int i=0; i<repeat; i++) {
// 		setJointSeq(velocity, valueJnt);
// 		setJointSeq(velocity, flingJnt2);
// 	}
// 	setJointSeq(velocity, valueJnt);
// }


void BinPickingControl::doBinPickingPlanning(int LR_flag, Vector3 pickP, Vector3 pickR, Vector3 placeP)
{
	double bufferH = 0.30;
	setMotionSeq(LR_flag, 1.2, Vector3(pickP(0),pickP(1),bufferH), RadConv(pickR));
	setMotionSeq(LR_flag, 0.8, Vector3(pickP(0),pickP(1),pickP(2)+0.02), RadConv(pickR));
	setMotionSeq(LR_flag, 0.8, pickP, RadConv(pickR));
	
	setHandStateSeq(LR_flag, HAND_CLOSING);

	setMotionSeq(LR_flag, 0.8, Vector3(pickP(0),pickP(1),bufferH), RadConv(pickR));
	setMotionSeq(LR_flag, 0.8, Vector3(placeP(0),placeP(1),bufferH), RadConv(pickR));
	setMotionSeq(LR_flag, 0.5, placeP, RadConv(pickR));

	setHandStateSeq(LR_flag, HAND_OPENING);
	setJointSeq(1.0, InitJnt);
}

void BinPickingControl::calcJointSeqWaveHand()
{
	string n;
	int LR_flag = 1;
	if (LR_flag == 0) {
		cout << "Wave left hand!" << endl;
		setMotionSeqDual(LR_flag, 1.0, Vector3(0.35, -0.3, 0.38), RadConv(Vector3(-180, 0, 180)));
		n = "R";
	}
	else if (LR_flag == 1) {
		cout << "Wave left hand!" << endl;
		setMotionSeqDual(LR_flag, 1.0, Vector3(0.30, 0.3, 0.41), RadConv(Vector3(-180, 0, 180)));
		setMotionSeqDual(1-LR_flag, 1.0, Vector3(0.30, -0.3, 0.30), RadConv(Vector3(-90, 0, 180)));
		n = "L";
	}

	double angleJnt3 = 4;
	double angleJnt4 = 25;
	int tmp_j = gc->bodyItemRobot()->body()->link("RARM_JOINT0")->jointId();

	int j3 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT0")->jointId();
	int j4 = gc->bodyItemRobot()->body()->link(n+"ARM_JOINT5")->jointId();
	int numArmJnt = gc->arm(LR_flag)->arm_path->numJoints();
	
	VectorXd valueJnt(numJnt);
	VectorXd flingJnt1(numJnt);
	VectorXd flingJnt2(numJnt);
	VectorXd iflingJnt1(numJnt);
	VectorXd iflingJnt2(numJnt);

	VectorXd _flingJnt1(numJnt);

	for (int i=0; i<numJnt; i++) {
		// valueJnt(i) = gc->graspMotionSeq.back().jointSeq(i);
		flingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		flingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		_flingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q();

		iflingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		iflingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q();
		if (i == j3) {
			flingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(angleJnt3);
			flingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q() - RadConv(angleJnt3);
			_flingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(angleJnt3)/2;
			iflingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(angleJnt3);
			iflingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q() - RadConv(angleJnt3);

		}
		else if (i==j4) {
			flingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(angleJnt4);
			flingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q() - RadConv(angleJnt4);
			_flingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(angleJnt4)/2;
			iflingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(angleJnt4);
			iflingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q() - RadConv(angleJnt4);
			
		}
		else if (i==tmp_j) {
			// flingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() - RadConv(10);
			// flingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(10);
			iflingJnt1(i) = gc->bodyItemRobot()->body()->joint(i)->q() - RadConv(20);
			// iflingJnt2(i) = gc->bodyItemRobot()->body()->joint(i)->q() + RadConv(10);
		}
	}
	// totalT unit: second	
	// double totalT = 10;
	double totalT = 7200;

	double T = 0.7;
	int repeat = totalT/(T*2);
	// int repeat = 300;
	os << "I'm going to wave my hand for " <<floor(totalT/60)<<"m"<< fmod(totalT,60) <<"s! " << endl;
	
	for (int i=0; i<repeat; i++) {
		if (i==0) {
			setJointSeq(T, _flingJnt1);
			setJointSeq(T, flingJnt2);
		}
		// if (i%4 == 0){
		// 	setJointSeq(T, iflingJnt1);
		// 	setJointSeq(T, iflingJnt2);
		// 	// setJointSeq(T, iflingJnt2);

		// }
		else {
			setJointSeq(T, flingJnt1);
			setJointSeq(T, flingJnt2);

		}
		i++;
	}
	
}
// modified/simplified from PRM/TrajectoryPlanning.cpp
bool BinPickingControl::trajectoryPlanning() 
{

	// init for trajectory planning 
	string name = "GraspPoseSeqItem0";
	poseSeqItemRobot = new PoseSeqItem();
	poseSeqItemRobot->setName(name);
	gc->bodyItemRobot()->addSubItem(poseSeqItemRobot);	/* modified by qtconv.rb 4th rule*/
	// if (motionSeq.size() != 0) motionSeq.clear();
#ifdef DEBUG_MODE
	cout << "[DEBUG] Before planning: motionSeq.size()=" << motionSeq.size() << ", gc->graspMotionSeq.size()=" << gc->graspMotionSeq.size() << endl; 
#endif
	
	bool success;

	if(gc->graspMotionSeq.size() < 2){
		return false;
	}

	gc->setGraspingState(PlanBase::NOT_GRASPING);
	gc->setGraspingState2(PlanBase::NOT_GRASPING);
	MotionState tempState = gc->getMotionState();
	gc->initialCollision();

	vector<MotionState> inputMotionSeq;
	bool outputGraspMotionSeq = true;

	inputMotionSeq = gc->graspMotionSeq;

	grasp::mpkInterface planner(gc->robotBody(), gc->bodyItemEnv);

	vector<VectorXd> config, config_tmp;

	bool successAll=true;

	// register IK link index
	vector<int> baseindex_list;
	vector<int> eeindex_list;
	const Mapping& info = *gc->body()->info();
	const Listing& defaultIkLinks = *info.findListing("defaultIkInterpolationLinks");
	const Mapping& setupMap = *info.findMapping("defaultIKsetup");
	if(defaultIkLinks.isValid()){
		for(int i=0; i< defaultIkLinks.size(); ++i){
			if(setupMap.isValid()){
				const Listing& setup = *setupMap.findListing(gc->body()->link(defaultIkLinks[i])->name());
				if(setup.isValid() && !setup.empty()){
					baseindex_list.push_back(gc->body()->link(defaultIkLinks[i])->index());
					eeindex_list.push_back(gc->body()->link(setup[0].toString())->index());
				}
			}
		}
	}
	vector<vector<Vector3> > basepos;
	vector<vector<Vector3> > eepos;
	vector<vector<Matrix3> > baserpy;
	vector<vector<Matrix3> > eerpy;
	basepos.resize(baseindex_list.size());
	eepos.resize(eeindex_list.size());
	baserpy.resize(baseindex_list.size());
	eerpy.resize(eeindex_list.size());
	if(inputMotionSeq.size() > 1){
		gc->graspMotionSeq.clear();
		for(unsigned int i=0; i<inputMotionSeq.size()-1; i++){
			// cout <<"PRM: "<<  i <<" th input motion" << endl;
			config_tmp.clear();

			gc->setMotionState(inputMotionSeq[i]);
			//if(gc->graspMotionSeq[i].tolerance >=0) gc->setTolerance(gc->graspMotionSeq[i].tolerance);
			VectorXd cfull(inputMotionSeq[i].jointSeq.size()+6);
			cfull << inputMotionSeq[i].jointSeq, inputMotionSeq[i].pos, inputMotionSeq[i].rpy;
			config_tmp.push_back(cfull);

			cfull << inputMotionSeq[i+1].jointSeq, inputMotionSeq[i+1].pos, inputMotionSeq[i+1].rpy;
			config_tmp.push_back(cfull);
			std::string error_message = "";
			success = planner.call_planner(config_tmp, inputMotionSeq[i].pathPlanDOF);
			
			if (!success) {
				error_message = planner.error_message();
				os << " PRM: fail " << i << " th input motion (" << error_message << ")" << endl;
			}
			if(!success) successAll=false; 

			planner.call_smoother(config_tmp);

			for(int j=0;j<gc->robotBody()->numJoints();j++){
					gc->robotBody()->joint(j)->q() = inputMotionSeq[i+1].jointSeq[j];
			}
			gc->body()->link(0)->p() = inputMotionSeq[i+1].pos;
			gc->body()->link(0)->R() = rotFromRpy(inputMotionSeq[i+1].rpy);
			gc->calcForwardKinematics();

			vector<VectorXd> config_tmp2;

			for(unsigned int j=0; j<config_tmp.size()-1; j++){
				double dyaw = fabs( config_tmp[j][gc->robotBody()->numJoints()+5] - config_tmp[j+1][gc->robotBody()->numJoints()+5] ) ;
				int div = dyaw/3.0;
				div +=1;
				for(int i=0;i<div;i++){
					config_tmp2.push_back( config_tmp[j+1]*i/div + config_tmp[j]*(div-i)/div);
				}
			}
			config_tmp2.push_back(config_tmp.back());

			for(unsigned int j=0; j<config_tmp2.size(); j++){
				int l=i+1;
				if(j==0) l=i;

				gc->body()->link(0)->p() = config_tmp2[j].segment<3>(gc->robotBody()->numJoints());
				gc->body()->link(0)->R() = rotFromRpy( config_tmp2[j].segment<3>(gc->robotBody()->numJoints()+3) );
				for(int k=0;k<gc->robotBody()->numJoints();k++) gc->robotBody()->joint(k)->q() = config_tmp2[j][k];
				gc->setInterLink();
				gc->calcForwardKinematics();
				MotionState temp = gc->getMotionState();
				motionSeq.push_back( temp );

				if(outputGraspMotionSeq){
					if (l < inputMotionSeq.size()) {
						temp.motionTime = inputMotionSeq[l].motionTime;
						temp.graspingState = inputMotionSeq[l].graspingState;
						temp.graspingState2 = inputMotionSeq[l].graspingState2;
					} 
					if(j < config_tmp2.size()-1 || i==inputMotionSeq.size()-2){
						gc->graspMotionSeq.push_back(temp);
					}
				}
			}
		}
	}
	else{
		return false;
	}

	if(baseindex_list.size()>0){
		for(unsigned int i=0; i<inputMotionSeq.size(); i++){
			for(int j=0;j<gc->robotBody()->numJoints();j++){
				gc->robotBody()->joint(j)->q() = inputMotionSeq[i].jointSeq[j];
			}
			gc->calcForwardKinematics();
			if(i > 0){   // initial pos/attitude is recorded only once
				// motion start position and attitude
				for(int k=0; k<baseindex_list.size(); ++k){
					basepos[k].push_back(gc->robotBody()->link(baseindex_list[k])->p());
					eepos[k].push_back(gc->robotBody()->link(eeindex_list[k])->p());
					baserpy[k].push_back(gc->robotBody()->link(baseindex_list[k])->R());
					eerpy[k].push_back(gc->robotBody()->link(eeindex_list[k])->R());
				}
			}
			// motion end position and attitude
			for(int k=0; k<baseindex_list.size(); ++k){
				basepos[k].push_back(gc->robotBody()->link(baseindex_list[k])->p());
				eepos[k].push_back(gc->robotBody()->link(eeindex_list[k])->p());
				baserpy[k].push_back(gc->robotBody()->link(baseindex_list[k])->R());
				eerpy[k].push_back(gc->robotBody()->link(eeindex_list[k])->R());
			}
		}
	}
	gc->setGraspingState(PlanBase::NOT_GRASPING);
	gc->setGraspingState2(PlanBase::NOT_GRASPING);

	gc->setMotionState(tempState);
	gc->robotBody()->calcForwardKinematics();

	PosePtr pose = new Pose(gc->bodyItemRobot()->body()->numJoints());
	for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
			pose->setJointPosition(gc->bodyItemRobot()->body()->joint(i)->jointId(), gc->bodyItemRobot()->body()->joint(i)->q());
	}

	vector<VectorXd>::iterator it = config.begin();
	double time = 0;
	vector<double> key_time;       // times specified in motion.dat
	for(int j=1;j<inputMotionSeq.size(); ++j){
		key_time.push_back(inputMotionSeq[j].startTime);   // command start time
		key_time.push_back(inputMotionSeq[j].endTime);         // command end time
	}
	for(int j=0;j<motionSeq.size();j++){
		PosePtr pose_ = new Pose(*pose);
		for(int k=0; k<baseindex_list.size(); ++k){
			pose_->setBaseLink(baseindex_list[k], basepos[k][j], baserpy[k][j]);
			pose_->setBaseLink(eeindex_list[k], eepos[k][j], eerpy[k][j]);
		}
		pose_->setBaseLink(0, motionSeq[j].pos, rotFromRpy(motionSeq[j].rpy) );
		for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
			pose_->setJointPosition(i, motionSeq[j].jointSeq[i]);
		}
		motionSeq[j].endTime = time;
		motionSeq[j].time = time;
		poseSeqItemRobot->poseSeq()->insert(poseSeqItemRobot->poseSeq()->end(), time , pose_);

		// time += key_time[j+1] - key_time[j];    // command run time or interval between commands
		time += 1.0;
	}
	poseSeqItemRobot->updateInterpolation();
	poseSeqItemRobot->updateTrajectory();
	ItemTreeView::mainInstance()->selectItem( poseSeqItemRobot->bodyMotionItem() );

	// updateTrajectoriesFromMotion(motionSeq, (gc->targetObject != NULL));
	BodyPtr robotBody = gc->bodyItemRobot()->body();
	const int numJointsRobot = robotBody->numJoints();

	BodyMotionPtr motionRobot = poseSeqItemRobot->bodyMotionItem()->motion();
	const double frameRate = motionRobot->frameRate();
	const int numFrames = motionRobot->getNumFrames();

	MultiValueSeqPtr qseqRobot = motionRobot->jointPosSeq();
	MultiSE3SeqPtr pseqRobot = motionRobot->linkPosSeq();

	// set robot to inital state
	vector<MotionState>::const_iterator itg = motionSeq.begin();
	gc->setHandObjectState( itg->handObjectState );
	gc->setGraspingState( itg->graspingState );
	gc->setGraspingState2( itg->graspingState2 );
	itg++;

	gc->setMotionState(*itg);

	for (int frame = 0; frame < numFrames; ++frame) {
		if( (itg != motionSeq.end()) && ( itg->time <= ((double)frame)/frameRate) ){
			gc->setHandObjectState( itg->handObjectState );
			gc->robotBody()->calcForwardKinematics();
			gc->setGraspingState( itg->graspingState );
			gc->setGraspingState2( itg->graspingState2 );
			itg++;
		}

		// set robot
		SE3& p_robot = pseqRobot->at(frame, 0);
		robotBody->link(0)->p() = p_robot.translation();
	
		robotBody->link(0)->R() = Matrix3(p_robot.rotation());

		for (int i = 0; i < numJointsRobot; ++i) {
				robotBody->joint(i)->q() = qseqRobot->at(frame, i);
		}

		gc->calcForwardKinematics();
	}
#ifdef DEBUG_MODE
	cout << "[DEBUG] After planning: motionSeq.size()=" << motionSeq.size() << ", gc->graspMotionSeq.size()=" << gc->graspMotionSeq.size() << endl; 
	// cout << "[DEBUG] Display sequence " << endl;
	// for (int i=0; i<motionSeq.size();i++) {
	// 	cout << "#" <<i<<": "<< "("<<motionSeq[i].motionTime<<")"<<"(" <<motionSeq[i].graspingState<<motionSeq[i].graspingState2 << ") ";
	// 	for (int j=0; j<PlanBase::instance()->bodyItemRobot()->body()->numJoints(); j++)
	// 	{
	// 		cout << motionSeq[i].jointSeq[j]*180/M_PI << ' ';
	// 	}
	// 	cout << '\n';
	// }
	// cout << "-----------------------------------------------" << endl;
#endif
	return successAll;
}