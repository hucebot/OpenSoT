#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <OpenSoT/utils/resources_utils.h>

namespace py = pybind11;

void pyResourcesUtils(py::module& m) {
    m.doc() = "OpenSoT resources utility functions";

    // root() -> returns std::filesystem::path, map to str in Python
    m.def("root", []() {
        return OpenSoT::resources_utils::root().string();
    }, "Get the root path of OpenSoT resources");

    // find(filename) -> returns optional path, map to Optional[str]
    m.def("find", [](const std::string& filename) -> py::object {
        auto p = OpenSoT::resources_utils::find(filename);
        if(p.has_value())
            return py::cast(p->string());
        else
            return py::none();
    }, "Find a file in the OpenSoT resources directory");

    // ReadFile(path) -> returns string
    m.def("ReadFile", &OpenSoT::resources_utils::ReadFile,
          "Read the entire file content as a string");
}
