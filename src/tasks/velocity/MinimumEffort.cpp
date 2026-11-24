/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;


MinimumEffort::MinimumEffort(const XBot::ModelInterface& robot_model, const double step) :
    Task("min_effort", robot_model.getNv()),
    _model(robot_model),
    _gTauGradientWorker(robot_model),
    _step(step)
{
    _W.resize(robot_model.getNv(), robot_model.getNv());
    _W.setIdentity();

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _gradient.resize(_model.getNv());
    _deltas.resize(_model.getNv());

    /* first update. Setting desired pose equal to the actual pose */
    this->_update();
}

MinimumEffort::~MinimumEffort()
{

}

void MinimumEffort::_update() {
    _q = _model.getJointPosition();

    /************************* COMPUTING TASK *****************************/

    _gradient.setZero();
    _deltas.setZero();


    for(unsigned int i = 0; i < _gradient.size(); ++i)
    {
        if(this->getActiveJointsMask()[i])
        {
            _deltas[i] = _step;
            double fun_a = _gTauGradientWorker.compute(_model.sum(_q, _deltas));
            double fun_b = _gTauGradientWorker.compute(_model.sum(_q, -_deltas));

            _gradient[i] = (fun_a - fun_b)/(2.0*_step);
            _deltas[i] = 0.0;
        } else
            _gradient[i] = 0.0;
    }

    _b = -1.0 * _lambda * _gradient;

    /**********************************************************************/
}

double MinimumEffort::computeEffort()
{
    return _gTauGradientWorker.compute(_model.getJointPosition());
}



