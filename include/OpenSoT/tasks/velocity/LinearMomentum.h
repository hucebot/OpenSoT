/*
 * Copyright (C) 2017 Walkman
 * Authors: Enrico Mingo Hoffman, Pouya Mohammadi
 * email:  enrico.mingo@iit.it, p.mohammadi@tu-bs.de
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __TASKS_VELOCITY_LINEAR_MOMENTUM_H__
#define __TASKS_VELOCITY_LINEAR_MOMENTUM_H__

#include <OpenSoT/Task.h>
#include <xbot2_interface/xbotinterface2.h>
#include <Eigen/Dense>

namespace OpenSoT {
   namespace tasks {
       namespace velocity {

    #define BASE_LINK_COM "world"
    #define DISTAL_LINK_COM "CoM"

       /**
        * @brief The LinearMomentum class implement the tracking of a desired linear momentum:        
        *
        *               \f$ \mathbf{A} \Delta \mathbf{q} = \mathbf{h}_d \text{dT}  \f$
        *
        * where \f$ \mathbf{h}_d \f$ is the desired linear momentum at the CoM
        * @note This is basically a copy of Enrico Mingo's AngularMomentum with
        * one difference that it takes the linear part of centroidal momentum matrix.
        */
       class LinearMomentum : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
       public:
           typedef std::shared_ptr<LinearMomentum> Ptr;

       private:
           XBot::ModelInterface& _robot;

           Eigen::Vector3d _desiredLinearMomentum;

           Eigen::MatrixXd _Momentum;

           void _update();

        public:
           /**
            * @brief LinearMomentum constructor
            * @param x joint states
            * @param robot reference to a model
            */
           LinearMomentum(XBot::ModelInterface& robot);
           ~LinearMomentum();

           /**
            * @brief setReference set a desired linear momentum at CoM
            * @param desiredLinearMomentum vector 3x1
            * NOTE: the input desired linear momentum has to be multiplied by \f$ \text{dT} \f$!
            */
           void setReference(const Eigen::Vector3d& desiredLinearMomentum);

           /**
            * @brief getReference get the desired linear momentum at CoM
            * @param desiredLinearMomentum vector 3x1
            */
           void getReference(Eigen::Vector3d& desiredLinearMomentum) const;

           /**
            * @brief getBaseLink
            * @return "world"
            */
           std::string getBaseLink();

           /**
            * @brief getDistalLink
            * @return "CoM"
            */
           std::string getDistalLink();

           /**
            * @brief isLinearMomentum
            * @param task a pointer to a Task
            * @return true if the task is LinearMomentum
            */
           static bool isLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           /**
            * @brief asLinearMomentum
            * @param task a pointer to a Task
            * @return a pointer to an LinearMomentum Task
            */
           static OpenSoT::tasks::velocity::LinearMomentum::Ptr asLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
       };
    }
   }
}

#endif
