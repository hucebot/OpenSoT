#include <OpenSoT/oc/TorquesConstraint.h>

using namespace OpenSoT::oc;

DynamicsConstraint::DynamicsConstraint(XBot::ModelInterface &robot,
                                       const AffineHelper &dX,
                                       const AffineHelper &dU) : Constraint("DynamicsConstraint", robot.getNv()),
                                                                 _robot(robot),
                                                                 _dU(dU),
                                                                 _dX(dX),
                                                                 _task(robot, dX, dU)
{

    _torquelim = _robot.getEffortLimits();

    update();
}

void DynamicsConstraint::_update()
{
    _task.update();

    _Aineq = _task.getA();

    _bLowerBound = -_torquelim + _task.getb(); // + bcause of the deffinition inside the task
    _bUpperBound = _torquelim + _task.getb();  // + bcause of the deffinition inside the task
}

void DynamicsConstraint::addForce(const std::string& frame_name, const std::shared_ptr<AffineHelper> force)
{
    _task.addForce(frame_name, force);
}


Eigen::VectorXd DynamicsConstraint::getTorqueLimit(){
    return _torquelim;
}

void DynamicsConstraint::setTorqueLimit(Eigen::VectorXd tau)
{
    _torquelim = tau;
}


