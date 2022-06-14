#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid{
void exportPlanBase(py::module m);
void exportGrasp(py::module m);
// void exportAssemblyObject(py::module m);
}
PYBIND11_MODULE(GraspPlugin, m)
{
	m.doc() = "Choreonoid GraspPlugin module";

	py::module::import("cnoid.Base");
	py::module::import("cnoid.Body");
	py::module::import("cnoid.BodyPlugin");

	exportPlanBase(m);
	exportGrasp(m);
	// exportAssemblyObject(m);
}

