#include "../ColdetPairData.h"
#include "../PlanBase.h"

#include <cnoid/PyBase>
#include <cnoid/BodyItem>

using namespace grasp;
using namespace cnoid;
namespace py = pybind11;

namespace {

PlanBase* PlanBase_Instance(PlanBase* pb = NULL) {return grasp::PlanBase::instance(pb);}

bool Set_grasping_robot(PlanBase& self, cnoid::ItemPtr item, int arm_id = 0) {
    cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    bool ret = true;
    if (!(self.targetArmFinger) || (self.bodyItemRobot()->body() != bodyitem->body())) {
	ret = self.SetGraspingRobot(bodyitem);
	if (ret) self.targetArmFinger = self.armsList[arm_id];
    }
    return ret;
}

void Select_arm(PlanBase& self, int arm_id) {
    self.targetArmFinger = self.armsList[arm_id];
}

void Set_grasped_object(PlanBase& self, cnoid::ItemPtr item) {
    cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    self.SetGraspedObject(bodyitem);
}

bool Append_assemble_object(PlanBase& self, cnoid::ItemPtr item) {
    cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
    return self.AppendAssembleObject(bodyitem);
}
}

namespace cnoid {
void exportPlanBase(py::module m) {

    py::class_ <PlanBase> (m, "PlanBase")
        .def(py::init<>())
        .def("graspMoionSeq", &PlanBase::getGraspingStateMainArm);

	m.def("instance", &PlanBase_Instance, py::arg("pb")=NULL);
        m.def("setRobot", &Set_grasping_robot, py::arg("self"), py::arg("item"), py::arg("arm_id")=0);
	m.def("selectArm", &Select_arm);
	m.def("setObject", &Set_grasped_object);
	m.def("Append_assemble_object", &Append_assemble_object);
}
}
