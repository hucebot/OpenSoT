#include <OpenSoT/oc/EulerVector.h>

using namespace OpenSoT::oc;

EulerVector::EulerVector(const XBot::ModelInterface& robot,
                               const AffineHelper& dX,
                               const AffineHelper& dU,
                               std::shared_ptr<AffineHelper> Xk,
                               std::shared_ptr<AffineHelper> Uk,
                               std::shared_ptr<AffineHelper> Xk_1,
                               const double dt):
    Task< Eigen::MatrixXd, Eigen::VectorXd> ("EulerVector", dX.getInputSize()),
    _robot(robot),
    _dU(dU),
    _dX(dX),
    _Xk(Xk),
    _Uk(Uk),
    _Xk_1(Xk_1),
    _dt(dt)
{

    _Fx = Eigen::MatrixXd::Identity(_dX.getOutputSize(), _dX.getOutputSize());
    _Fu = Eigen::MatrixXd::Identity(_dU.getOutputSize(), _dU.getOutputSize()) * _dt;

    _dXnext = _Fx * _dX + _Fu * _dU;
    _A = _dXnext.getM();

    _W.setIdentity(dX.getOutputSize(), dX.getOutputSize());

    update();
}

void EulerVector::_update()
{  
    _b = (_Xk->getValue() + _Uk->getValue()*_dt) - _Xk_1->getValue();
}
