#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <OpenSoT/solvers/hpipmOC.h>
#include <OpenSoT/solvers/swSQP.h>

std::string print_opt(OpenSoT::solvers::swSQP::options &opt)
{
    std::string str;
    str = opt.toOSS().str();
    return str;
}


namespace py = pybind11;
using namespace OpenSoT;


void pyHPIPMOC(py::module& m) {
    py::enum_<hpipm::HpipmMode>(m, "HpipmMode")
        .value("SpeedAbs", hpipm::HpipmMode::SpeedAbs)
        .value("Speed",    hpipm::HpipmMode::Speed)
        .value("Balance",  hpipm::HpipmMode::Balance)
        .value("Robust",   hpipm::HpipmMode::Robust)
        .export_values();

    py::class_<hpipm::OcpQpIpmSolverSettings>(m, "OcpQpIpmSolverSettings")
        .def(py::init<>())
        .def_readwrite("mode",        &hpipm::OcpQpIpmSolverSettings::mode)
        .def_readwrite("iter_max",    &hpipm::OcpQpIpmSolverSettings::iter_max)
        .def_readwrite("alpha_min",   &hpipm::OcpQpIpmSolverSettings::alpha_min)
        .def_readwrite("mu0",         &hpipm::OcpQpIpmSolverSettings::mu0)
        .def_readwrite("tol_stat",    &hpipm::OcpQpIpmSolverSettings::tol_stat)
        .def_readwrite("tol_eq",      &hpipm::OcpQpIpmSolverSettings::tol_eq)
        .def_readwrite("tol_ineq",    &hpipm::OcpQpIpmSolverSettings::tol_ineq)
        .def_readwrite("tol_comp",    &hpipm::OcpQpIpmSolverSettings::tol_comp)
        .def_readwrite("reg_prim",    &hpipm::OcpQpIpmSolverSettings::reg_prim)
        .def_readwrite("warm_start",  &hpipm::OcpQpIpmSolverSettings::warm_start)
        .def_readwrite("pred_corr",   &hpipm::OcpQpIpmSolverSettings::pred_corr)
        .def_readwrite("ric_alg",     &hpipm::OcpQpIpmSolverSettings::ric_alg)
        .def_readwrite("split_step",  &hpipm::OcpQpIpmSolverSettings::split_step)
        .def("checkSettings",         &hpipm::OcpQpIpmSolverSettings::checkSettings);

    py::class_<hpipm::OcpQpSolution>(m, "OcpQpSolution")
    .def(py::init<>())  // default constructor
        .def_readwrite("x", &hpipm::OcpQpSolution::x)
        .def_readwrite("u", &hpipm::OcpQpSolution::u)
        .def_readwrite("pi", &hpipm::OcpQpSolution::pi)
        .def_readwrite("P", &hpipm::OcpQpSolution::P)
        .def_readwrite("p", &hpipm::OcpQpSolution::p)
        .def_readwrite("K", &hpipm::OcpQpSolution::K)
        .def_readwrite("k", &hpipm::OcpQpSolution::k);

    py::class_<solvers::hpipmOC, std::shared_ptr<solvers::hpipmOC>>(m, "hpipmOC")
        .def(py::init<const unsigned int>())
        .def("setBoundsX", &solvers::hpipmOC::setBoundsX)
        .def("setBoundsU", &solvers::hpipmOC::setBoundsU)
        .def("setConstraint", &solvers::hpipmOC::setConstraint)
        .def("setFullCost", &solvers::hpipmOC::setFullCost)
        .def("setCost", &solvers::hpipmOC::setCost)
        .def("setLSCost", &solvers::hpipmOC::setLSCost,
        py::arg("i"), py::arg("Ax"), py::arg("Wx"), py::arg("bx"), py::arg("Au") = Eigen::MatrixXd(0,0), py::arg("Wu") = Eigen::MatrixXd(0,0), py::arg("bu") = Eigen::VectorXd(0))
        .def("solve", &solvers::hpipmOC::solve)
        .def("getSolution", &solvers::hpipmOC::getSolution)
        .def("setStageDynamics", &solvers::hpipmOC::setStageDynamics)
        .def("getOptions", &solvers::hpipmOC::getOptions, py::return_value_policy::reference_internal);

    // Bind swSQP::options
    py::class_<OpenSoT::solvers::swSQP::options>(m, "swSQPOptions")
        .def(py::init<>())
        .def("print", print_opt)
        .def_readwrite("verbose", &OpenSoT::solvers::swSQP::options::verbose)
        .def_readwrite("max_iters", &OpenSoT::solvers::swSQP::options::max_iters)
        .def_readwrite("alpha_min", &OpenSoT::solvers::swSQP::options::alpha_min)
        .def_readwrite("beta", &OpenSoT::solvers::swSQP::options::beta)
        .def_readwrite("min_abs_delta_solution", &OpenSoT::solvers::swSQP::options::min_abs_delta_solution)
        .def_readwrite("line_search_strategy", &OpenSoT::solvers::swSQP::options::line_search_strategy)
        .def_readwrite("hessian_scale_factor_up", &OpenSoT::solvers::swSQP::options::hessian_scale_factor_up)
        .def_readwrite("initial_hessian_regularization", &OpenSoT::solvers::swSQP::options::initial_hessian_regularization)
        .def_readwrite("max_hessian_regularization", &OpenSoT::solvers::swSQP::options::max_hessian_regularization);

    // Bind swSQP::stage_statistics
    py::class_<OpenSoT::solvers::swSQP::stage_statistics>(m, "swSQPStageStatistics")
        .def(py::init<>())
        .def_readonly("cost", &OpenSoT::solvers::swSQP::stage_statistics::cost)
        .def_readonly("constraint_violation", &OpenSoT::solvers::swSQP::stage_statistics::constraint_violation);

    // Bind swSQP::statistics
    py::class_<OpenSoT::solvers::swSQP::statistics>(m, "swSQPStatistics")
        .def(py::init<const unsigned int>())
        .def_readonly("cost", &OpenSoT::solvers::swSQP::statistics::cost)
        .def_readonly("constraint_violation", &OpenSoT::solvers::swSQP::statistics::constraint_violation)
        .def_readonly("iters", &OpenSoT::solvers::swSQP::statistics::iters)
        .def_readonly("alpha", &OpenSoT::solvers::swSQP::statistics::alpha)
        .def_readonly("line_search_iters", &OpenSoT::solvers::swSQP::statistics::line_search_iters)
        .def_readonly("line_search_accepted", &OpenSoT::solvers::swSQP::statistics::line_search_accepted)
        .def_readonly("iter_time", &OpenSoT::solvers::swSQP::statistics::iter_time)
        .def_readonly("total_time", &OpenSoT::solvers::swSQP::statistics::total_time)
        .def_readonly("stages_statistics", &OpenSoT::solvers::swSQP::statistics::stages_statistics)
        .def("toString", [](OpenSoT::solvers::swSQP::statistics &self) -> std::string { return self.toOSS().str();});

    // Bind swSQP
    py::class_<OpenSoT::solvers::swSQP, OpenSoT::solvers::swSQP::Ptr>(m, "swSQP")
        .def(py::init<OpenSoT::ocp::Ptr>(), py::arg("ocp"))
        .def("init", &OpenSoT::solvers::swSQP::init)
        .def("solve", &OpenSoT::solvers::swSQP::solve)
        .def("getStateSolution", &OpenSoT::solvers::swSQP::getStateSolution)
        .def("getControlSolution", &OpenSoT::solvers::swSQP::getControlSolution)
        .def("getOptions", (OpenSoT::solvers::swSQP::options & (OpenSoT::solvers::swSQP::*)()) & OpenSoT::solvers::swSQP::getOptions, py::return_value_policy::reference_internal)
        .def("getQPSolver", &OpenSoT::solvers::swSQP::getQPSolver)
        .def("getStatistics", &OpenSoT::solvers::swSQP::getStatistics, py::return_value_policy::reference_internal);

}
