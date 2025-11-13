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

    // update();
}

void TorquesTask::_update()
{
    // _robot.computeInverseDynamicsDerivative(_dtau_dq, _dtau_dv, _dtau_da);


    _dtau_dq.setZero();
    _dtau_dv.setZero();
    _dtau_da.setZero();
    _Fx.setZero();
    _Fu.setZero();

    if (_fext_flag)
    {
        for (const auto& [frame_name, dforce_var] : _frame_forces_vars)
        {
            std::cout<<frame_name << " : dforce_var.getValue() = "<< dforce_var.getValue()<< std::endl;
            _frame_forces[frame_name] = dforce_var.getValue();
        }
        
        std::cout<< "mplaaaaaa"<< std::endl;
        _robot.computeInverseDynamicsDerivative(_dtau_dq, _dtau_dv, _dtau_da, _dtau_dfext, _frame_forces);

        // _dTAU =  _robot.computeInverseDynamics(_frame_forces);
        // for (const auto& [frame_name, dforce_var] : _frame_forces_vars)
        //     _dTAU += _dtau_dfext[frame_name] * dforce_var;

    }
    else
    {
        _robot.computeInverseDynamicsDerivative(_dtau_dq, _dtau_dv, _dtau_da);
        // _dTAU = _robot.computeInverseDynamics();
    }

    _Fx.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_dq;
    _Fx.block(0, _robot.getNv(), _robot.getNv(), _robot.getNv()) = _dtau_dv;

    _Fu.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_da;

    _dTAU = _Fx * _dX + _Fu * _dU + _robot.computeInverseDynamics(_frame_forces);

    _A = _dTAU.getM();
    _b = -_dTAU.getq();
}

void TorquesTask::addForce(const std::string& frame_name, const AffineHelper& force)
{
    _fext_flag = true;
    _frame_forces_vars[frame_name] = force;
    std::cout<<frame_name << " : force = "<< force<< std::endl;
    _dtau_dfext[frame_name] = Eigen::MatrixXd::Zero(6, _robot.getNv());
}