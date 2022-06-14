#include <iostream>

#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <boost/python.hpp>

#include "../PlanBase.h"
#include "../PlanInterface.h"

using namespace cnoid;
using namespace grasp;
namespace py = pybind11;

namespace {
int add(int i, int j) {
    return i + j;
}
int add(int i){
    return i + 1994;
}
void say() {
    std::cout << "Hello! " << std::endl;
}
}

namespace cnoid{
void exportGrasp(py::module m)
{
    m.doc() = "pybind11 example plugin";
    // m.def("add", &add, "A function that adds two integers");
    m.def("add", py::overload_cast<int,int>(&add));
    m.def("add", py::overload_cast<int>(&add));
    m.def("say", &say, "say");
}
}
//PYBIND11_MODULE(GraspPlugin, m) {
//    m.doc() = "pybind11 example plugin"; // optional module docstring

//    m.def("add", &add, "A function that adds two numbers");
//}

