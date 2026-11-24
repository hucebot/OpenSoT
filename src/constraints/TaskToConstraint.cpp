/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/constraints/TaskToConstraint.h>

#include <assert.h>
#include <limits>

using namespace OpenSoT::constraints;

TaskToConstraint::TaskToConstraint(TaskPtr task) :
    BilateralConstraint(task->getTaskID(), task->getA(), task->getb(), task->getb()),
    _task(task),
    _err_lb(Eigen::VectorXd::Zero(task->getTaskSize())),
    _err_ub(Eigen::VectorXd::Zero(task->getTaskSize()))
{
    this->generateAll();
}

TaskToConstraint::TaskToConstraint(TaskToConstraint::TaskPtr task, 
                                   const Eigen::VectorXd& err_lb, 
                                   const Eigen::VectorXd& err_ub): 
    BilateralConstraint(task->getTaskID(), task->getA(), task->getb() + err_lb, task->getb() + err_ub),
    _task(task),
    _err_lb(err_lb),
    _err_ub(err_ub)
{
    
    if( ((err_ub-err_lb).array() < 0).any() ){
        throw std::runtime_error("Some components of err_ub are smaller than err_lb!!!");
    }
    
    if( err_lb.size() != task->getA().rows() || err_ub.size() != task->getA().rows() ){
        throw std::runtime_error("Either err_ub or err_lb has wrong dimension!!!");
    }
    
    this->generateAll();
}


void TaskToConstraint::_update()
{
    _task->update();
    this->generateAll();
}

void TaskToConstraint::generateAll() {
    
    _Aineq = _task->getA();
    _bLowerBound = _task->getb() + _err_lb;
    _bUpperBound = _task->getb() + _err_ub;
    

    assert( (_Aineq.rows() == _bLowerBound.rows()) &&
            (_Aineq.rows() == _bUpperBound.rows()));
}
