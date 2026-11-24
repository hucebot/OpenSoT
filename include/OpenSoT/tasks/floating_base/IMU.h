/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef _OPENSOT_TASK_FLOATING_BASE_IMU_
#define _OPENSOT_TASK_FLOATING_BASE_IMU_

#include <OpenSoT/Task.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT{
    namespace tasks{
        namespace floating_base{
        /**
         * @brief The IMU class estimates the orientation velocities of the floating base from the
         * angular velocities of the IMU.
         * NOTE: we consider the IMU attached to the floating_base link
         */
        class IMU: public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{
            public:
                typedef std::shared_ptr<IMU> Ptr;
            /**
                 * @brief IMU accept a robot model and an imu, throws an error if the imu is
                 * not attached to the floating_base
                 * @param robot
                 * @param imu
                 */
                IMU(XBot::ModelInterface& robot, XBot::ImuSensor::ConstPtr imu);
                ~IMU();

                void _update();

            private:
                Eigen::MatrixXd _J;
                XBot::ModelInterface& _robot;
                std::string _fb_link;
                XBot::ImuSensor::ConstPtr _imu;
                Eigen::Vector3d _angular_velocity;
        };

        }
    }
}

#endif
