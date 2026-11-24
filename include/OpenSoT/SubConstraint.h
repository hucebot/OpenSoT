/*
 * Copyright (C) 2021 Multidof
 * Author: Enrico Mingo Hoffman
 * email:  enricomingo@gmail.com
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __SUBCONSTRAINT_H__
#define __SUBCONSTRAINT_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Indices.h>
#include <Eigen/Dense>
#include <list>
#include <string>
#include <cassert>
#include <memory>

namespace OpenSoT {

class SubConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
public:

    typedef std::shared_ptr<OpenSoT::SubConstraint> Ptr;

    /**
     * @brief SubConstraint create a SubConstraint object by specifying the father Consatrint through a pointer,
     * and a list of row indices. Notice the row indices start from 0 (c style)
     * @param constrPtr a pointer to the father constraint
     * @param rowIndices a list of indices. The index to the first row is 0.
     */
    SubConstraint(ConstraintPtr constrPtr, const std::list<unsigned int> rowIndices);

    virtual ~SubConstraint(){}

protected:
    Indices _subConstraintMap;
    ConstraintPtr _constraintPtr;
    void _update();

    static const std::string _SUBCONSTRAINT_SEPARATION_;

    void generateBound(const Eigen::VectorXd& bound, Eigen::VectorXd& sub_bound);
    void generateConstraint(const Eigen::MatrixXd& A, Eigen::MatrixXd& sub_A);


};

}


#endif
