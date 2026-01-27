#include <OpenSoT/oc/PosSO3Constraint.h>

using namespace OpenSoT::oc;

PosSO3Constraint::PosSO3Constraint(const XBot::ModelInterface &robot,
                                   const AffineHelper &dX,
                                   const std::string &distal_frame) : Constraint("PosSO3Constraint", dX.getInputSize()),
                                                                      _robot(robot),
                                                                      _dX(dX),
                                                                      _task("PosSO3Task_Constraint", robot, dX, distal_frame)
{

    _upper_lim = 1e12 * Eigen::Vector6d::Ones();
    _lower_lim = -1e12 * Eigen::Vector6d::Ones();

    update();
}

void PosSO3Constraint::_update()
{
    _task.update();

    _Aineq = _task.getA();

    _bLowerBound = _lower_lim + _task.getb(); // + bcause of the deffinition inside the task
    _bUpperBound = _upper_lim + _task.getb(); // + bcause of the deffinition inside the task
}

void PosSO3Constraint::setUpperLimits(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rotation)
{
    _upper_lim.head(3) = pos;
}

void PosSO3Constraint::setLowerLimits(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rotation)
{
    _lower_lim.head(3) = pos;
}
