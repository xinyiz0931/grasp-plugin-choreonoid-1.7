#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {
void exportControl(py::module m);
}

PYBIND11_MODULE(MotionFile, m)
{
    m.doc() = "blah";

    py::module::import("cnoid.Base");
    py::module::import("cnoid.Body");
    py::module::import("cnoid.BodyPlugin");

    exportControl(m);
}