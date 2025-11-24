#include <OpenSoT/oc/TorquesTask.h>

using namespace OpenSoT::oc;

TorquesTask::TorquesTask(XBot::ModelInterface &robot,
                                       const AffineHelper &dX,
                                       const AffineHelper &dU) : Task<Eigen::MatrixXd, Eigen::VectorXd>("TorquesTask", dX.getInputSize()),
                                                                _robot(robot),
                                                                _dU(dU),
                                                                _dX(dX)
{

    _W.setIdentity(_robot.getNv(), _robot.getNv());

    _Fx.resize(_robot.getNv(), _dX.getOutputSize());
    _Fu.resize(_robot.getNv(), _dU.getOutputSize());

    _dtau_dq.resize(robot.getNv(), robot.getNv());
    _dtau_dv.resize(robot.getNv(), robot.getNv());
    _dtau_da.resize(robot.getNv(), robot.getNv());

    _fext_flag = false;

    _A.setZero(_robot.getNv(), _dX.getInputSize());
    _b.setZero(_robot.getNv());

    update();
}

void TorquesTask::_update()
{
    _dtau_dq.setZero();
    _dtau_dv.setZero();
    _dtau_da.setZero();
    _Fx.setZero();
    _Fu.setZero();


    for (const auto& [frame_name, force_var] : _frame_forces_vars)
    {
        _frame_forces[frame_name].head(3) = force_var->getValue();
        _frame_forces[frame_name].tail(3) << 0., 0., 0.;
        _dtau_dfext[frame_name].setZero();
    }
    _robot.computeInverseDynamicsDerivative(_dtau_dq, _dtau_dv, _dtau_da, _dtau_dfext, _frame_forces);
    
    int i = 0;
    for (const auto& [frame_name, force_var] : _frame_forces_vars)
    {   
        auto f_idx = force_var->getStartIdx() - _dX.getOutputSize();
        if(_robot.isFloatingBase())
            f_idx = f_idx-1;

        _Fu.block(0, f_idx, _robot.getNv(), force_var->getOutputSize()) = -_dtau_dfext[frame_name].topRows(3).transpose();
        i++;
    }

    
    _Fx.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_dq;
    _Fx.block(0, _robot.getNv(), _robot.getNv(), _robot.getNv()) = _dtau_dv;

    _Fu.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_da;

    //_dTAU = _Fx * _dX + _Fu * _dU + _robot.computeInverseDynamics(_frame_forces);


    _A.leftCols(_dX.getOutputSize()) = _Fx;
    _A.rightCols(_dU.getOutputSize()) = _Fu;
    _b = -1. * _robot.computeInverseDynamics(_frame_forces);

    //_A = _dTAU.getM();
    //_b = -_dTAU.getq();
}

void TorquesTask::addForce(const std::string& frame_name, const std::shared_ptr<VariableXd> force)
{
    _frame_forces_vars[frame_name] = force;
    _dtau_dfext[frame_name] = Eigen::MatrixXd::Zero(6, _robot.getNv());
}
