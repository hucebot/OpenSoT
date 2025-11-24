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

        _Aineq.resize(1, dX.getInputSize()) ;
        _bLowerBound.resize(1);
        _bUpperBound.resize(1);

        update();
    }

    void FrictionConeConstraint::_update()
    {

        _d_T_w = _robot.getPose(_distal_frame).inverse();
        _J.setZero();
        _Adj.setZero();
        _Aineq.setZero();
        _bLowerBound.setZero();
        _bUpperBound.setZero();

        _w_force = _robot.getPose(_distal_frame).linear() * _force_var->getValue();

        _robot.getJacobian(_distal_frame, _J); // LOCAL_WORLD_ALIGNED
        _Adj.block<3,3>(0,0) = _d_T_w.linear();
        _Adj.block<3,3>(3,3) = _d_T_w.linear();
        _J = _Adj * _J; // LOCAL


        auto f_idx = _force_var->getStartIdx();
        if(_robot.isFloatingBase())
            f_idx = f_idx-1;

        Eigen::MatrixXd dwf_dq = -_robot.getPose(_distal_frame).linear() * hat(_force_var->getValue()) * _J.bottomRows(3);
        
        // _Aineq.block(0,_force_var->getStartIdx(), _force_var->getOutputSize(), _force_var->getOutputSize()) = Eigen::Matrix3d::Identity();
        _Aineq.block(0, 0, 1, _robot.getNv()) = dwf_dq.row(2);
        _Aineq.block(0, f_idx, 1, _force_var->getOutputSize()) = _robot.getPose(_distal_frame).linear().col(2).transpose();
        
        // _Aineq = _task.getA();
        // _bLowerBound.topRows(3) = - _force_var->getValue();
        // _bUpperBound.topRows(3) = Eigen::Vector3d::Identity() * std::numeric_limits<double>::infinity();

        _bLowerBound(0) = - _w_force(2) ;
        _bUpperBound(0) = 1000000 - _w_force(2);
    }





}
