#include <OpenSoT/oc/FrictionConeConstraint.h>
#include <OpenSoT/utils/LieGroupsUtils.h>

namespace OpenSoT::oc {

    FrictionConeConstraint::FrictionConeConstraint(XBot::ModelInterface &robot,
                                        const std::string& frame_name,
                                        const std::shared_ptr<VariableXd> force,
                                        const AffineHelper &dX,
                                        const AffineHelper &dU) :   Constraint("FrictionConeConstraint", dX.getInputSize()),
                                                                    _robot(robot),
                                                                    _distal_frame(frame_name),
                                                                    _force_var(force)
    {

        _Aineq.resize(5, dX.getInputSize()) ;
        _bLowerBound.resize(5);
        _bUpperBound.resize(5);

        _mu = 1.;

        _inf = 100000.;

        update();
    }

    void FrictionConeConstraint::_update()
    {

        _wTd = _robot.getPose(_distal_frame);
        _J.setZero();
        _Adj.setZero();
        _Aineq.setZero();
        _bLowerBound.setZero();
        _bUpperBound.setZero();

        _w_force = _wTd.linear() * _force_var->getValue();

        _robot.getJacobian(_distal_frame, _J); // LOCAL_WORLD_ALIGNED
        _Adj.block<3,3>(0,0) = _wTd.linear().transpose();
        _Adj.block<3,3>(3,3) = _wTd.linear().transpose();
        _J = _Adj * _J; // LOCAL


        auto f_idx = _force_var->getStartIdx();
        if(_robot.isFloatingBase())
            f_idx = f_idx-1;

        Eigen::MatrixXd dwf_dq = -_wTd.linear() * hat(_force_var->getValue()) * _J.bottomRows(3);
        
        _Aineq.block(0, 0, 1, _robot.getNv()) = dwf_dq.row(2);
        _Aineq.block(0, f_idx, 1, _force_var->getOutputSize()) = _wTd.linear().col(2).transpose();

        _Aineq.block(1, 0, 1, _robot.getNv()) = _mu * dwf_dq.row(2) - dwf_dq.row(0);
        _Aineq.block(1, f_idx, 1, _force_var->getOutputSize()) =  (_mu * _wTd.linear().col(2) - _wTd.linear().col(0)).transpose();

        _Aineq.block(2, 0, 1, _robot.getNv()) = _mu * dwf_dq.row(2) + dwf_dq.row(0);
        _Aineq.block(2, f_idx, 1, _force_var->getOutputSize()) =  (_mu * _wTd.linear().col(2) + _wTd.linear().col(0)).transpose();

        _Aineq.block(3, 0, 1, _robot.getNv()) = _mu * dwf_dq.row(2) - dwf_dq.row(1);
        _Aineq.block(3, f_idx, 1, _force_var->getOutputSize()) =  (_mu * _wTd.linear().col(2) - _wTd.linear().col(1)).transpose();

        _Aineq.block(4, 0, 1, _robot.getNv()) = _mu * dwf_dq.row(2) + dwf_dq.row(1);
        _Aineq.block(4, f_idx, 1, _force_var->getOutputSize()) =  (_mu * _wTd.linear().col(2) + _wTd.linear().col(1)).transpose();
        

        _bLowerBound(0) = - _w_force(2);
        _bUpperBound(0) = _inf;

        _bLowerBound(1) = -(_mu * _w_force(2) - _w_force(0));
        _bUpperBound(1) = _inf-(_mu * _w_force(2) - _w_force(0));

        _bLowerBound(2) = -(_mu * _w_force(2) + _w_force(0));
        _bUpperBound(2) = _inf-(_mu * _w_force(2) - _w_force(0));

        _bLowerBound(3) = -(_mu * _w_force(2) - _w_force(1));
        _bUpperBound(3) = _inf-(_mu * _w_force(2) - _w_force(1));

        _bLowerBound(4) = -(_mu * _w_force(2) + _w_force(1));
        _bUpperBound(4) = _inf-(_mu * _w_force(2) - _w_force(1));


    }





}
