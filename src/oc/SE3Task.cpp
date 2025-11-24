#include <OpenSoT/oc/SE3Task.h>


using namespace OpenSoT::oc;

SE3Task::SE3Task(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame, const ReferenceFrame reference_frame):
Task(id, dx.getInputSize()),
_robot(robot),
_dx(dx),
_distal_frame(distal_frame),
_reference_frame(reference_frame)
{
    _W.setIdentity(dx.getOutputSize(), dx.getOutputSize());

    _ref = _robot.getPose(_distal_frame);

    _J.resize(6, _robot.getNv());
    _J.setZero();
    __A = _J;

    _A.setZero(6, _dx.getInputSize());
    _b.setZero(6);

    update();
}

void SE3Task::_update()
{   
    _d_T_w = _robot.getPose(_distal_frame).inverse();
    _J.setZero();

    if(_reference_frame == ReferenceFrame::WORLD)
    {
        /**
         * Here we compute the spatial Jacobian in WORLD from the classic Jacobian in world computed by the model interface.
         *
         * NOTE: we pass the translation from local to world because inside the getJacobian(frame, p, J)
         * first rotate p in world and then apply the skew.
         **/
        _robot.getJacobian(_distal_frame, _d_T_w.translation(), _J); // equivalent to call pinocchio::Jacobian() in WORLD (meaning: the point is in WORLD, velocities are expressed in WORLD)
    }
    
    if(_reference_frame == ReferenceFrame::LOCAL)
    {
        /**
         * We now compute the Adjoint to rotate to LOCAL, equivalent to call pinocchio::Jacobian() in LOCAL (meaning: the point is in LOCAL, velocities are expressed in LOCAL)
        **/
         _Adj.setZero();

        _robot.getJacobian(_distal_frame, _J); //here we take the Jacobian LOCAL_WORLD_ALIGNED (meaning: the point is in the LOCAL and the velocities are expressed in WORLD)
        _Adj.block<3,3>(0,0) = _d_T_w.linear();
        _Adj.block<3,3>(3,3) = _d_T_w.linear();

        _J = _Adj * _J;
    }

    _error = _d_T_w * _ref;
    _w = Log6(_error);

    J_l6_inv(_w, _J_l6_inv);
    if(_reference_frame == ReferenceFrame::LOCAL)
        __A =  -_J_l6_inv * _J; // This should be the derivative of _w when using LOCAL...
    else
    {
        adjoint(_d_T_w, _Adj);
        __A =  -_J_l6_inv * _Adj * _J; // This should be the derivative of _w when using WORLD...
    }

    //_task = __A*_dx + _w;

    _A.leftCols(_dx.getOutputSize()) = __A;
    _b = -_w;
    
    //_A = _task.getM();
    //_b = -_task.getq();
}

 
const Eigen::Vector6d& SE3Task::getError()
{   
    return _w;
}
