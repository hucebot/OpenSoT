/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/


#ifndef __CONSTRAINT_GENERIC_CONSTRAINT_H__
#define __CONSTRAINT_GENERIC_CONSTRAINT_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT { namespace constraints  {
    
class GenericConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
  
public:
    
    enum class Type {
        /** The GenericConstraint is of the type:
          *     lb <= x <= ub
         **/
        BOUND,
        /** The GenericConstraint is of the type:
         *      L <= Ax <= U
         **/
        CONSTRAINT
    };
    
    typedef std::shared_ptr<GenericConstraint> Ptr;

    /**
     * @brief GenericConstraint specific constructor for BOUND type
     * @param constraint_id
     * @param upper_bound
     * @param lower_bound
     */
    GenericConstraint(std::string constraint_id,
                      const Eigen::VectorXd& upper_bound,
                      const Eigen::VectorXd& lower_bound,
                      const int x_size);
    
    GenericConstraint(std::string constraint_id,
                      const AffineHelper& variable,
                      const Eigen::VectorXd& upper_bound,
                      const Eigen::VectorXd& lower_bound,
                      const Type constraint_type
                      );

    bool setConstraint(const AffineHelper& var,
                       const Eigen::VectorXd& upper_bound,
                       const Eigen::VectorXd& lower_bound);
    
    bool setBounds(const Eigen::VectorXd& upper_bound,
                   const Eigen::VectorXd& lower_bound);

    Type getType(){return _type;}
    
    virtual void _update();
    
private:
    
        AffineHelper _var;
        
        Eigen::VectorXd _ub, _lb;
        
        Type _type;

    
};
    
} } 



#endif
