/*
 * Copyright (C) 2014 Walkman
 * Authors:Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __TASKS_VELOCITY_CONTACT_H__
#define __TASKS_VELOCITY_CONTACT_H__

 #include <OpenSoT/Task.h>
 #include <xbot2_interface/xbotinterface2.h>
 #include <Eigen/Dense>

 #define WORLD_FRAME_NAME "world"


namespace OpenSoT { namespace tasks {  namespace velocity {

class Contact : public Task <Eigen::MatrixXd, Eigen::VectorXd> {

public:

    typedef std::shared_ptr<Contact> Ptr;

    Contact(std::string task_id,
            const XBot::ModelInterface& model,
            std::string link_name,
            const Eigen::MatrixXd& contact_matrix
           );

    ~Contact();
    

    void _update();

    const std::string& getLinkName() const;

    bool baseLinkIsWorld() const;

    /**
        * @brief getError returns the 6d cartesian error (position and orientation) between actual and reference pose
        * @return a \f$R^{6}\f$ vector describing cartesian error between actual and reference pose
        */
    const Eigen::VectorXd& getError() const;

protected:

    const XBot::ModelInterface& _model;

    std::string _distal_link;

    int _distal_link_index;

    bool _base_link_is_world;

    void update_b();

    bool _is_initialized;

    Eigen::VectorXd _error;

    Eigen::MatrixXd _K, _Jtmp, _Jrot;

};

 } } }

#endif
