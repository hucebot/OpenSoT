/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef _OPENSOT_FLOATING_BASE_CONTACT_ESTIMATION_
#define _OPENSOT_FLOATING_BASE_CONTACT_ESTIMATION_

#include <OpenSoT/Task.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT{
    namespace tasks{
        namespace floating_base{
            /**
             * @brief The Contact class estimates floating_base velocities (linear and angular)
             * from the joint velocities. We consider a link in contact with the environment which
             * does not move wrt the world
             */
            class Contact: public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{
            public:
                typedef std::shared_ptr<Contact> Ptr;

                /**
                 * @brief Contact constructor which accept a robot model, a link name which represent
                 * the link in contact and a contact matrix (default is Identity)
                 * @param robot
                 * @param link_in_contact
                 * @param contact_matrix
                 */
                Contact(XBot::ModelInterface& robot, 
                        const std::string& link_in_contact,
                        const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd::Identity(6,6), 
                        const Eigen::Affine3d& desired_contact_pose = Eigen::Affine3d::Identity() );
                ~Contact();
                virtual void _update();

                void setLinkInContact(const std::string link_in_contact);
                const std::string& getLinkInContact() const;

            private:
                std::string _link_in_contact;
                XBot::ModelInterface& _robot;
                Eigen::MatrixXd _J, _Jrot;
                Eigen::MatrixXd _Jcontact;
                Eigen::VectorXd _dqm;
                Eigen::MatrixXd _contact_matrix;
                Eigen::Affine3d _w_T_c_des;
            };
        }
    }
}

#endif
