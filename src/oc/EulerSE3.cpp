#include <OpenSoT/oc/EulerSE3.h>

using namespace OpenSoT::oc;

EulerSE3::EulerSE3(const XBot::ModelInterface& robot,
                               const AffineHelper& dX,
                               const AffineHelper& dU,
                               std::shared_ptr<AffineHelper> Xk,
                               std::shared_ptr<AffineHelper> Uk,
                               std::shared_ptr<AffineHelper> Xk_1,
                               const double dt):
    Task< Eigen::MatrixXd, Eigen::VectorXd> ("EulerSE3", dX.getInputSize()),
    _robot(robot),
    _dU(dU),
    _dX(dX),
    _Xk(Xk),
    _Uk(Uk),
    _Xk_1(Xk_1),
    _dt(dt)
{
    if(_dX.getOutputSize() != 6)
        throw std::runtime_error("_dX != 6");

    if(_dU.getOutputSize() != 6)
        throw std::runtime_error("_dU != 6");


    _W.setIdentity(dX.getOutputSize(), dX.getOutputSize());

    _b.setZero(dX.getOutputSize());
    _A.setZero(dX.getOutputSize(), dX.getInputSize());

    update();
}

void EulerSE3::_update()
{  
    _Uk->update();
    _Xk->update();
    _Xk_1->update();

    Eigen::Affine3d xk   = XYZQUATtoSE3(_Xk->getValue().segment<7>(0));
    Eigen::VectorXd uk   = _Uk->getValue().segment<6>(0);
    Eigen::Affine3d xk_1 = XYZQUATtoSE3(_Xk_1->getValue().segment<7>(0));
    Exp6(uk*_dt, _Exp6);
    Eigen::Affine3d diff =  xk_1.inverse() * xk * _Exp6;

    _Fx = J_r6_inv(Log6(diff)) * JMaMb_Ma(xk, _Exp6);
    _Fu = J_r6_inv(Log6(diff)) * JMaMb_Mb(xk, _Exp6) * JExp6(_Exp6) * _dt;

    _A.leftCols(6) = _Fx;
    _A.middleCols(_robot.getNv(), 6) = _Fu;

    _b = Log6(diff);
}
