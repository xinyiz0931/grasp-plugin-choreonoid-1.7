#include <iostream>

#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/PyUtil>

#include "../../Grasp/PlanBase.h"
#include "../../Grasp/PlanInterface.h"

using namespace grasp;
namespace py = pybind11;

namespace {

bool TestSetRobot(cnoid::BodyItemPtr bodyitem, int arm_id=0) {
    return false;
}
}

namespace cnoid {
void exportControl(py::module m) {
    m.def("test_set_robot", &TestSetRobot, py::arg("bodyitem"), py::arg("arm_id")=0);
}
}