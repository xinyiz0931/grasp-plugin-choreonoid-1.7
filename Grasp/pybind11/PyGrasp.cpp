#include <iostream>

#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/PyUtil>
#include <pybind11/stl.h> // vector
#include <pybind11/operators.h>//operator

#include "../PlanBase.h"
#include "../PlanInterface.h"

using namespace std;
using namespace grasp;
using namespace cnoid;
namespace py = pybind11;
namespace 
{
void Grasp() {
    bool init = PlanBase::instance()->initial();
    if (!init) {
        MessageView::instance()->cout() << "Failed: Grasp Planning Initial" << std::endl;
	return;
    }
    try {
        PlanInterface::instance()->doGraspPlanning();
    }
    catch(int number) {
        PlanBase::instance()->stopFlag = false;
	    MessageView::instance()->cout() << "Grasp Planning is stopped" <<std::endl;
    }
    PlanBase::instance()->flush();
}
void Set_joint_position(BodyItemPtr bodyitem, vector<double> qs) {
    
    int nj = bodyitem->body()->numAllJoints();
    if(qs.size() != nj){
        if(qs.size() != bodyitem->body()->numJoints()){
            MessageView::instance()->cout() << "Mismatched size of the stored joint positions for " << bodyitem->name() << std::endl;
        }
        nj = std::min(int(qs.size()), nj);
    }
    for(int i=0; i < nj; ++i){
        bodyitem->body()->joint(i)->q() = qs[i];
    } 
    bodyitem->body()->calcForwardKinematics();
    bodyitem->notifyKinematicStateChange();
}

bool Set_grasping_robot(BodyItemPtr bodyitem, int arm_id=0) {
    //  BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    bool ret = true;
    if (!(PlanBase::instance()->targetArmFinger) || (PlanBase::instance()->bodyItemRobot()->body() != bodyitem->body())) {
        ret = PlanBase::instance()->SetGraspingRobot(bodyitem);
	if (ret) PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[arm_id];
    }
    return ret;
}

// void Set_grasped_object(ItemPtr item) {
void Set_grasped_object(BodyItemPtr bodyitem) {
    // BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    PlanBase::instance()->SetGraspedObject(bodyitem);
}
}

namespace cnoid{
void exportGrasp(py::module m)
{
    m.doc() = "choreonoid grasp plugin";
    m.def("grasp", &Grasp);
    m.def("set_robot", &Set_grasping_robot, py::arg("bodyitem"), py::arg("arm_id")=0);
    m.def("set_joint_position", &Set_joint_position); 
    m.def("set_object", &Set_grasped_object);
}
}
