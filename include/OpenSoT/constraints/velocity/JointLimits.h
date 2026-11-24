/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __BOUNDS_VELOCITY_JOINTLIMITS_H__
#define __BOUNDS_VELOCITY_JOINTLIMITS_H__

 #include <OpenSoT/Constraint.h>
 #include <xbot2_interface/xbotinterface2.h>


 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The JointLimits class implements bounds on joints positions
             */
            class JointLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<JointLimits> Ptr;
            private:
                const XBot::ModelInterface& _robot;
                double _boundScaling;
                Eigen::VectorXd _jointLimitsMin;
                Eigen::VectorXd _jointLimitsMax;
                Eigen::VectorXd _dq;

                void _update();
            public:
                /**
                 * @brief JointLimits constructor
                 * @param q the configuration of the robot when
                 *          creating the joint limits constraint
                 * @param jointBoundMax upper bounds for joint limits
                 * @param jointBounMin lower bounds for joint limits
                 * @note bounds are intended as min/max motions that can be
                 * applied for unit time starting from the robot neutral configuration
                 * i.e. if q0 = robot.getNeutralQ(), then qf = robot.sum(q0, v) is
                 * within joint limits iff jointBoundMax <= v <= jointBoundMax
                 */
                JointLimits(const XBot::ModelInterface& robot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const double boundScaling = 1.0);


                void setBoundScaling(const double boundScaling);
            };
        }
    }
 }

#endif
