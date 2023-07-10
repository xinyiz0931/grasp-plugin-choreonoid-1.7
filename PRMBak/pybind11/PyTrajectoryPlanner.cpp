#include <iostream>
#include <fstream>
#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>

#include "../TrajectoryPlanner.h"
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
bool Set_Test(cnoid::BodyItemPtr bodyitem, int arm_id=0) {
    //  cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    bool ret = true;
    if (!(PlanBase::instance()->targetArmFinger) || (PlanBase::instance()->bodyItemRobot()->body() != bodyitem->body())) {
        ret = PlanBase::instance()->SetGraspingRobot(bodyitem);
	if (ret) PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[arm_id];
    }
	cnoid::MessageView::instance()->cout() << "Finish! " << endl;
    return ret;
}
void Plan_Motion() {
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
// void Load_Motion_File() {
// 	cout << "initial pose (xinyi printing)"  <<endl;
// 	cout << PlanBase::instance()->bodyItemRobot()->body()->link(0)->p().transpose() <<endl;
	
// 	static MotionFileControl* instance = new MotionFileControl();
// 	string motionfile;
// 	motionfile = "./ext/bpbot/data/motion/motion.dat";
// 	FILE *fp;

// 	if( (fp = fopen((motionfile).c_str(), "r")) != NULL ){
// 		MotionFileControl::instance()->LoadFromMotionFile(motionfile);
// 		fclose(fp);
// 	}
// 	cnoid::MessageView::instance()->cout() << "Loaded motion file" << endl;
// }
}

namespace cnoid {
void exportPlanning(py::module m) {

	m.doc() = "pybind11 motion planning";
	m.def("add", &add);
	m.def("set_test", &Set_Test, py::arg("bodyitem"), py::arg("arm_id")=0);
	m.def("plan_motion", &Plan_Motion);
	// m.def("load_file", &Load_Motion_File);

}
}
