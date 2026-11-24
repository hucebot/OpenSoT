/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __OPENSOT_ACCELERATION_TASK_CONTACT_H__
#define __OPENSOT_ACCELERATION_TASK_CONTACT_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/common/utils.h>


namespace OpenSoT { namespace tasks { namespace acceleration {
    
    class Contact : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
        
    public:
        
        typedef std::shared_ptr<Contact> Ptr;

        Contact(const std::string& task_id,
                const XBot::ModelInterface& robot,
                const std::string& contact_link,
                const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd()
                );
        
        Contact(const std::string& task_id,
                const XBot::ModelInterface& robot, 
                const std::string& contact_link,
                const AffineHelper& qddot,
                const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd()
                 );
        

        virtual void _update();
        
        virtual void _log(XBot::MatLogger2::Ptr logger);
        
    private:
        
        std::string _contact_link;
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _contact_task;
        
        Eigen::MatrixXd _J, _K;
        Eigen::Vector6d _jdotqdot;
        
    };
    
} } }









#endif
