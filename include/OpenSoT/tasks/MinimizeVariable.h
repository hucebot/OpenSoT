/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/


#ifndef __CONSTRAINT_MINIMIZE_VARIABLE_H__
#define __CONSTRAINT_MINIMIZE_VARIABLE_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT { namespace tasks  {
    
class MinimizeVariable : public Task<Eigen::MatrixXd, Eigen::VectorXd> {
  
public:
    
    typedef std::shared_ptr<MinimizeVariable> Ptr;
    
    MinimizeVariable(std::string task_id, 
                      const AffineHelper& variable
                      );
    
    bool setReference(const Eigen::VectorXd& ref);
    
    virtual void _update();

    void getReference(Eigen::VectorXd& ref) const;
    
    
    
private:
    
        AffineHelper _var;
        Eigen::VectorXd _ref;
    
};
    
} } 





#endif
