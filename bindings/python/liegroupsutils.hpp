#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include <OpenSoT/utils/LieGroupsUtils.h>


namespace py = pybind11;


void pyopensot_lie_groups(py::module &m)
{
      // Hat and unhat operators
      m.def("hat", &OpenSoT::hat, 
            "Hat operator: R^3 -> so(3)",
            py::arg("v"));

      m.def("unhat", &OpenSoT::unhat,
            "Vee (unhat) operator: so(3) -> R^3", 
            py::arg("M"));

      // SO(3) exponential and logarithm
      m.def("Exp3", &OpenSoT::Exp3,
            "Exponential map: so(3) -> SO(3)",
            py::arg("p"));

      m.def("Log3", &OpenSoT::Log3,
            "Logarithm map: SO(3) -> so(3)",
            py::arg("R"));

      // SO(3) Jacobians
      m.def("J_l", &OpenSoT::J_l,
            "Left Jacobian of SO(3)",
            py::arg("theta"));

      m.def("J_l_inv", &OpenSoT::J_l_inv,
            "Inverse left Jacobian of SO(3)",
            py::arg("theta"));

      // SE(3) exponential and logarithm
      m.def("Exp6", py::overload_cast<const Eigen::Vector6d&>(&OpenSoT::Exp6),
            "Exponential map: se(3) -> SE(3)",
            py::arg("tau"));

      m.def("Log6", &OpenSoT::Log6,
            "Logarithm map: SE(3) -> se(3)",
            py::arg("T"));

      // Adjoint
      m.def("Adjoint", &OpenSoT::Adjoint,
            "Adjoint representation of SE(3)",
            py::arg("M"));

      // SE(3) Jacobians
      m.def("J_l6", py::overload_cast<const Eigen::Vector6d&>(&OpenSoT::J_l6),
            "Left Jacobian for SE(3)",
            py::arg("tau"));

      m.def("J_l6_inv", py::overload_cast<const Eigen::Vector6d&>(&OpenSoT::J_l6_inv),
            "Inverse left Jacobian for SE(3)",
            py::arg("tau"));

      m.def("J_r6", py::overload_cast<const Eigen::Vector6d&>(&OpenSoT::J_r6),
            "Right Jacobian for SE(3)",
            py::arg("tau"));

      m.def("J_r6_inv", py::overload_cast<const Eigen::Vector6d&>(&OpenSoT::J_r6_inv),
            "Inverse left Jacobian for SE(3)",
            py::arg("tau"));

      m.def("JExp6", &OpenSoT::JExp6,
            "Jacobian of Exp6",
            py::arg("M"));

      m.def("JLog6", &OpenSoT::JLog6,
            "Jacobian of Log6",
            py::arg("M"));

      // Composition Jacobians
      m.def("JMaMb_Ma", &OpenSoT::JMaMb_Ma,
            "Jacobian of Ma*Mb w.r.t. Ma",
            py::arg("Ma"), py::arg("Mb"));

      m.def("JMaMb_Mb", &OpenSoT::JMaMb_Mb,
            "Jacobian of Ma*Mb w.r.t. Mb",
            py::arg("Ma"), py::arg("Mb"));

      m.def("JMinv_M", &OpenSoT::JMinv_M,
            "Jacobian of M^-1 w.r.t. M",
            py::arg("M"));

      // Quaternion conversions
      m.def("QUATtoSO3", &OpenSoT::QUATtoSO3,
            "Convert quaternion to rotation matrix",
            py::arg("q"));

      m.def("SO3toQUAT", &OpenSoT::SO3toQUAT,
            "Convert rotation matrix to quaternion",
            py::arg("R"));

      m.def("XYZQUATtoSE3", &OpenSoT::XYZQUATtoSE3,
            "Convert [x,y,z,qx,qy,qz,qw] to SE(3)",
            py::arg("q"));

      m.def("SE3toXYZQUAT", &OpenSoT::SE3toXYZQUAT,
            "Convert SE(3) to [x,y,z,qx,qy,qz,qw]",
            py::arg("M"));
}
