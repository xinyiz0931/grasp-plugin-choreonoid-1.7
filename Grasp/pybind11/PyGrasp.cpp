#include <iostream>

#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/PyUtil>

#include "../PlanBase.h"
#include "../PlanInterface.h"

using namespace grasp;
namespace py = pybind11;

namespace 
{
void Grasp() {
    bool init = PlanBase::instance()->initial();
    if (!init) {
        cnoid::MessageView::instance()->cout() << "Failed: Grasp Planning Initial" << std::endl;
	return;
    }
    try {
        PlanInterface::instance()->doGraspPlanning();
    }
    catch(int number) {
        PlanBase::instance()->stopFlag = false;
	    cnoid::MessageView::instance()->cout() << "Grasp Planning is stopped" <<std::endl;
    }
    PlanBase::instance()->flush();
}

bool Set_grasping_robot(cnoid::BodyItemPtr bodyitem, int arm_id=0) {
    //  cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    bool ret = true;
    if (!(PlanBase::instance()->targetArmFinger) || (PlanBase::instance()->bodyItemRobot()->body() != bodyitem->body())) {
        ret = PlanBase::instance()->SetGraspingRobot(bodyitem);
	if (ret) PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[arm_id];
    }
    return ret;
}

// void Set_grasped_object(cnoid::ItemPtr item) {
void Set_grasped_object(cnoid::BodyItemPtr bodyitem) {
    // cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    PlanBase::instance()->SetGraspedObject(bodyitem);
}
}

namespace cnoid{
void exportGrasp(py::module m)
{
    m.doc() = "choreonoid grasp plugin";
    m.def("grasp", &Grasp);
    m.def("set_robot", &Set_grasping_robot, py::arg("bodyitem"), py::arg("arm_id")=0);
    
    m.def("set_object", &Set_grasped_object);
}
}
