/*
 * Copyright (C) 2017 Walkman
 * Author: Enrico Mingo Hoffman, Pouya Mohammadi
 * email:  enrico.mingo@iit.it, p.mohammadi@tu-bs.de
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/tasks/velocity/LinearMomentum.h>

using namespace OpenSoT::tasks::velocity;

LinearMomentum::LinearMomentum(XBot::ModelInterface& robot):
    Task("LinearMomentum", robot.getNv()), _robot(robot)
{
    _desiredLinearMomentum.setZero();
    this->_update();

    _W.resize(3,3);
    _W.setIdentity(3,3);

    _hessianType = HST_SEMIDEF;

    _Momentum.resize(6, _x_size);
    _A.resize(3, _x_size);
    _b = _desiredLinearMomentum;
}

LinearMomentum::~LinearMomentum()
{

}

void LinearMomentum::_update()
{
    _robot.computeCentroidalMomentumMatrix(_Momentum);
    _A = _Momentum.block(0,0,3,_x_size);
    _b = _desiredLinearMomentum;
}

void LinearMomentum::setReference(const Eigen::Vector3d& desiredLinearMomentum)
{
    _desiredLinearMomentum = desiredLinearMomentum;
}



void LinearMomentum::getReference(Eigen::Vector3d& desiredLinearMomentum) const
{
    desiredLinearMomentum = _desiredLinearMomentum;
}



std::string LinearMomentum::getBaseLink()
{
    return BASE_LINK_COM;
}

std::string LinearMomentum::getDistalLink()
{
    return DISTAL_LINK_COM;
}

LinearMomentum::Ptr LinearMomentum::asLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<LinearMomentum>(task);
}


bool LinearMomentum::isLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)std::dynamic_pointer_cast<LinearMomentum>(task);
}
