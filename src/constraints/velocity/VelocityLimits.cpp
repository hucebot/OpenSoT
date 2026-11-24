/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <cmath>

using namespace OpenSoT::constraints::velocity;

VelocityLimits::VelocityLimits(const XBot::v2::ModelInterface &robot, const double qDotLimit,
                               const double dT) :
    Constraint("velocity_limits", robot.getNv()), _dT(dT) {

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);
}

VelocityLimits::VelocityLimits(const XBot::v2::ModelInterface &robot, const Eigen::VectorXd& qDotLimit,
                               const double dT) :
    Constraint("velocity_limits", robot.getNv()), _dT(dT)
{
    if(qDotLimit.size() != _x_size)
        throw std::runtime_error("qDotLimit.size() != _x_size()");

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);
}

Eigen::VectorXd OpenSoT::constraints::velocity::VelocityLimits::getVelocityLimits()
{
    return _upperBound/_dT;
}

void OpenSoT::constraints::velocity::VelocityLimits::setVelocityLimits(const double qDotLimit)
{
    _qDotLimit = std::fabs(qDotLimit);

    this->generateBounds(_qDotLimit);
}

void OpenSoT::constraints::velocity::VelocityLimits::setVelocityLimits(const Eigen::VectorXd& qDotLimit)
{
    this->generateBounds(qDotLimit);
}

double OpenSoT::constraints::velocity::VelocityLimits::getDT()
{
    return _dT;
}

void VelocityLimits::generateBounds(const double qDotLimit)
{
    /************************ COMPUTING BOUNDS ****************************/
        _lowerBound<<_lowerBound.setOnes(_x_size)*-1.0*_qDotLimit*_dT;
        _upperBound<<_upperBound.setOnes(_x_size)*1.0*_qDotLimit*_dT;

    /**********************************************************************/
}

void VelocityLimits::generateBounds(const Eigen::VectorXd& qDotLimit)
{
    assert(qDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < qDotLimit.size(); ++i)
    {
        _lowerBound[i] = -1.0*std::fabs(qDotLimit[i])*_dT;
        _upperBound[i] = 1.0*std::fabs(qDotLimit[i])*_dT;
    }
}
