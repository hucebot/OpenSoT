#include <OpenSoT/oc/SE3VelTask.h>


using namespace OpenSoT::oc;

SE3VelTask::SE3VelTask(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame):
Task(id, dx.getInputSize()),
_robot(robot),
_dx(dx),
_distal_frame(distal_frame)
{
    _W.setIdentity(dx.getOutputSize(), dx.getOutputSize());

    _ref.setZero();

    _dV_dq.resize(6, robot.getNv());
    _dV_dv.resize(6, robot.getNv());

    _A.setZero(6, _dx.getInputSize());
    _b.setZero(6);

    update();
}

void SE3VelTask::_update()
{   
    _dV_dq.setZero();
    _dV_dv.setZero();

    _robot.getFrameVelocityDerivativesLocal(_distal_frame, _dV_dq, _dV_dv);

    _A.block(0, 0, 6, _robot.getNv()) = _dV_dq;
    _A.block(0, _robot.getNv(), 6, _robot.getNv()) = _dV_dv;

    _b = _ref - _robot.getFrameVelocityLocal(_distal_frame);
    
}