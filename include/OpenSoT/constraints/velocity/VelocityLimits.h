/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __BOUNDS_VELOCITY_VELOCITYLIMITS_H__
#define __BOUNDS_VELOCITY_VELOCITYLIMITS_H__

 #include <OpenSoT/Constraint.h>
#include <Eigen/Dense>
#include <xbot2_interface/xbotinterface2.h>


 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The VelocityLimits class implements a bound on joint velocities
             */
            class VelocityLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<VelocityLimits> Ptr;
            private:
                double _qDotLimit;
                double _dT;
            public:
                /**
                 * @brief VelocityLimits constructor
                 * @param qDotLimit the joint velocity limit. It is always a positive number [rad/s]
                 * @param dT the time constant at which we are performing velocity control [s]
                 * @param x_size the size of the unknowns that we want to bound (it CANNOT be a subset)
                 */
                VelocityLimits(const XBot::ModelInterface& robot,
                               const double qDotLimit,
                               const double dT);

                VelocityLimits(const XBot::ModelInterface& robot,
                               const Eigen::VectorXd& qDotLimit,
                               const double dT);

                /**
                 * @brief getVelocityLimits returns the current velocity limits.
                 * @return the joint velocity limits. It is always a positive double [rad/s]
                 */
                Eigen::VectorXd getVelocityLimits();

                /**
                 * @brief setVelocityLimits
                 * @param qDotLimit the joint velocity limits. It needs be a positive number [rad/s]
                 */
                void setVelocityLimits(const double qDotLimit);
                void setVelocityLimits(const Eigen::VectorXd& qDotLimit);

                /**
                 * @brief getDT returns the (constant) sample time we assume on the system.
                 * @return the system sample time in [s]
                 */
                double getDT();

            private:
                void generateBounds(const double qDotLimit);
                void generateBounds(const Eigen::VectorXd& qDotLimit);

                void _update(){}
            };
        }
    }
 }

#endif
