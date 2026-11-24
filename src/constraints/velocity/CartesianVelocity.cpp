/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/constraints/velocity/CartesianVelocity.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;

CartesianVelocity::CartesianVelocity(const Eigen::Vector6d velocityLimits,
                         			 const double dT,
                                     const OpenSoT::tasks::velocity::Cartesian::Ptr& task) :
Constraint(task->getTaskID()+"_VelocityLimit", task->getXSize()), _dT(dT), _velocityLimits(velocityLimits),
_cartesian_task(task), _is_cartesian(true) {

    _Aineq.resize(6,    task->getXSize());
    _bLowerBound.resize(3);
    _bUpperBound.resize(3);

    this->generatebBounds();

    this->generateAineq();
}

CartesianVelocity::CartesianVelocity(const Eigen::Vector3d velocityLimits,
                  const double dT,
                  const OpenSoT::tasks::velocity::CoM::Ptr &task) :
Constraint(task->getTaskID()+"_VelocityLimit", task->getXSize()), _dT(dT), _velocityLimits(velocityLimits),
_com_task(task), _is_cartesian(false)
{
    _Aineq.resize(3,    task->getXSize());
    _bLowerBound.resize(3);
    _bUpperBound.resize(3);

    this->generatebBounds();

    this->generateAineq();
}



void CartesianVelocity::_update() {

    this->generateAineq();

}

const Eigen::VectorXd &OpenSoT::constraints::velocity::CartesianVelocity::getVelocityLimits() const
{
    return _velocityLimits;
}

void OpenSoT::constraints::velocity::CartesianVelocity::setVelocityLimits(const Eigen::Vector6d &velocityLimits)
{
    _velocityLimits = velocityLimits;
}

void OpenSoT::constraints::velocity::CartesianVelocity::setVelocityLimits(const Eigen::Vector3d &velocityLimits)
{
    _velocityLimits = velocityLimits;
}

void CartesianVelocity::generatebBounds() {

    /******************** COMPUTING CONSTANT BOUNDS ***********************/

    _bLowerBound = -1.0*_velocityLimits*_dT;
    _bUpperBound = +1.0*_velocityLimits*_dT;

    /**********************************************************************/
}

void CartesianVelocity::generateAineq()
{
    /************************ COMPUTING BOUNDS ****************************/
    if(_is_cartesian)
        _Aineq = _cartesian_task->getA();
    else
        _Aineq = _com_task->getA();

    /**********************************************************************/
}

