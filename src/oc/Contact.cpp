#include <OpenSoT/oc/Contact.h>

#include <OpenSoT/utils/LieGroupsUtils.h>

namespace OpenSoT::oc {


    ContactConstraint::ContactConstraint(XBot::ModelInterface &robot,
                                        const std::string& frame_name,
                                        const AffineHelper &dX,
                                        const AffineHelper &dU) :   Constraint("ContactConstraint", dX.getInputSize()),
                                                                    _robot(robot),
                                                                    _frame_name(frame_name),
                                                                    _dU(dU),
                                                                    _dX(dX)

    {

        _Aineq.resize(4, dX.getInputSize()) ;
        _bLowerBound.resize(4);
        _bUpperBound.resize(4);

        _dvc_dq.resize(6, robot.getNv());
        _dvc_dv.resize(6, robot.getNv());
        _active = Eigen::VectorXd::Zero(3);

        deactivate();

        update();
    }

    void ContactConstraint::_update()
    {
        _wTd = _robot.getPose(_frame_name);
        _J.setZero();
        _Adj.setZero();
        _Aineq.setZero();
        _bLowerBound.setZero();
        _bUpperBound.setZero();

        _dvc_dq.setZero();
        _dvc_dv.setZero();


        _robot.getJacobian(_frame_name, _J); // LOCAL_WORLD_ALIGNED

        _robot.getFrameVelocityDerivativesLocal(_frame_name, _dvc_dq, _dvc_dv);
        _b = _robot.getFrameVelocityLocal(_frame_name).head(3);


        _Aineq.block(0, 0, 3, _robot.getNv()) = _dvc_dq.topRows(3);
        _Aineq.block(0, _robot.getNv(), 3, _robot.getNv()) = _dvc_dv.topRows(3);
        _Aineq.block(3,0,1,_robot.getNv()) = _J.row(2);

        

        _bLowerBound.head(3) = -_active - _b;
        _bUpperBound.head(3) = _active - _b;

        _bLowerBound(3) = _ground_height - _wTd.translation()(2) - 1e-2;
        _bUpperBound(3) = _active(2) + _ground_height - _wTd.translation()(2)+ 1e-2;

    }

    void ContactConstraint::activate(double ground_height){
        _active = Eigen::VectorXd::Zero(3);
        _ground_height = ground_height;
    }
    
    void ContactConstraint::deactivate(){
        _active = 100000000 * Eigen::VectorXd::Ones(3);
    }





}
