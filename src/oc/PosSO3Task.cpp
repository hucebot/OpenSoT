#include <OpenSoT/oc/PosSO3Task.h>


using namespace OpenSoT::oc;

PosSO3Task::PosSO3Task(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame):
Task(id, dx.getInputSize()),
_robot(robot),
_dx(dx),
_distal_frame(distal_frame)
{
    _W.setIdentity(dx.getOutputSize(), dx.getOutputSize());

    _ref = _robot.getPose(_distal_frame);

    _J.resize(6, _robot.getNv());
    _J.setZero();

    _A.setZero(6, _dx.getInputSize());
    _b.setZero(6);

    update();
}

void PosSO3Task::_update()
{   
    _wTd = _robot.getPose(_distal_frame);
    _J.setZero();

    _robot.getJacobian(_distal_frame, _J); // LOCAL_WORLD_ALIGNED
    // _Adj.block<3,3>(0,0) = _wTd.linear().transpose();
    // _Adj.block<3,3>(3,3) = _wTd.linear().transpose();
    // _J = _Adj * _J; // LOCAL

    _error.head(3) = _wTd.translation() - _ref.translation();
    _error.tail(3) = Log3(_wTd.linear().transpose() * _ref.linear());

    // J_l6_inv(_w, _J_l6_inv);
    
    _dT_dq = _J;
    _dT_dq.bottomRows(3) = - _wTd.linear().transpose() *  _dT_dq.bottomRows(3);

    _A.leftCols(_dx.getOutputSize()) = _dT_dq;
    _b = -_error;
    
}