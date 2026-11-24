/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/constraints/velocity/JointLimits.h>

using namespace OpenSoT::constraints::velocity;

JointLimits::JointLimits(   const XBot::ModelInterface& robot,
                            const Eigen::VectorXd& jointBoundMax,
                            const Eigen::VectorXd& jointBoundMin,
                            const double boundScaling) :
    Constraint("joint_limits", robot.getNv()),
    _robot(robot),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _boundScaling(boundScaling) {


    /* calling update to generate bounds */
    JointLimits::update();
}

void JointLimits::_update()
{
    _robot.difference(_robot.getJointPosition(),
                      _robot.getNeutralQ(),
                      _dq);

/************************ COMPUTING BOUNDS ****************************/

    if(_robot.getNv() == _jointLimitsMax.size())
    {
        _upperBound = ( _jointLimitsMax - _dq)*_boundScaling;
        _lowerBound = ( _jointLimitsMin - _dq)*_boundScaling;

        // avoid infeasibility
        _upperBound = _upperBound.cwiseMax(0.0);
        _lowerBound = _lowerBound.cwiseMin(0.0);
    }
    else
        XBot::Logger::warning("Wrong input x size, joint limits will not be upated!\n");

/**********************************************************************/

}

void JointLimits::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}




