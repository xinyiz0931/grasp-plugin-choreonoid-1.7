#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {
void exportPlanning(py::module m);
}

PYBIND11_MODULE(BinPicking, m)
{
	m.doc() = "Choreonoid PRM plugin  module";
	py::module::import("cnoid.Base");
	py::module::import("cnoid.Body");
	py::module::import("cnoid.BodyPlugin");

	exportPlanning(m);
}
