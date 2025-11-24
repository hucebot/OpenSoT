#include <OpenSoT/oc/Contact.h>


namespace OpenSoT::oc {


    ContactTask::ContactTask(XBot::ModelInterface &robot,
                                        const std::string& frame_name,
                                        const AffineHelper &dX,
                                        const AffineHelper &dU) : Task<Eigen::MatrixXd, Eigen::VectorXd>("ContactTask", dX.getInputSize()),
                                                                    _robot(robot),
                                                                    _frame_name(frame_name),
                                                                    _dU(dU),
                                                                    _dX(dX)
    {

        _W.setIdentity(6, 6);

        _Fx.resize(6, _dX.getOutputSize());
        _Fu.resize(6, _dU.getOutputSize());

        _dvc_dq.resize(6, robot.getNv());
        _dvc_dv.resize(6, robot.getNv());


        _A.setZero(6, _dX.getInputSize());
        _b.resize(6);

        update();
    }

    void ContactTask::_update()
    {

        _dvc_dq.setZero();
        _dvc_dq.setZero();
        _Fx.setZero();
        _Fu.setZero();


        _robot.getFrameVelocityDerivativesLocal(_frame_name, _dvc_dq, _dvc_dv);


        _Fx.block(0, 0, 6, _robot.getNv()) = _dvc_dq;
        _Fx.block(0, _robot.getNv(), 6, _robot.getNv()) = _dvc_dv;


        //_dcontact = _Fx * _dX + _Fu * _dU + _robot.getFrameVelocityLocal(_frame_name);

        //_A = _dcontact.getM();
        //_b = -_dcontact.getq();
        _A.leftCols(_dX.getOutputSize()) = _Fx;
        _A.rightCols(_dU.getOutputSize()) = _Fu;
        _b = -1. * _robot.getFrameVelocityLocal(_frame_name);
    }



    ContactConstraint::ContactConstraint(XBot::ModelInterface &robot,
                                        const std::string& frame_name,
                                        const AffineHelper &dX,
                                        const AffineHelper &dU) :   Constraint("ContactConstraint", dX.getInputSize()),
                                                                    _robot(robot),
                                                                    _task(robot, frame_name, dX, dU)
    {

        update();
    }

    void ContactConstraint::_update()
    {
        _task.update();

        _Aineq = _task.getA();

        _bLowerBound = _task.getb();
        _bUpperBound = _task.getb();
    }





}
