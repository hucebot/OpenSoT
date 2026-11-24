/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/constraints/BilateralConstraint.h>

#include <assert.h>
#include <limits>
#include <sstream>

using namespace OpenSoT::constraints;

int BilateralConstraint::_constr_count = 0;

BilateralConstraint::BilateralConstraint(const Eigen::MatrixXd &Aineq,
                                         const Eigen::VectorXd &bLowerBound,
                                         const Eigen::VectorXd &bUpperBound) :
    Constraint("bilateral_constr_", Aineq.cols())
{
    std::stringstream tmp; tmp << BilateralConstraint::_constr_count++;
    this->_constraint_id += tmp.str();

    _Aineq = Aineq;
    _bLowerBound = bLowerBound;
    _bUpperBound = bUpperBound;

    assert( (_Aineq.rows() == _bLowerBound.size()) &&
            (_Aineq.rows() == _bUpperBound.size()));
}


BilateralConstraint::BilateralConstraint(const std::string constraintName,
                                         const Eigen::MatrixXd &Aineq,
                                         const Eigen::VectorXd &bLowerBound,
                                         const Eigen::VectorXd &bUpperBound) :
    Constraint(constraintName, Aineq.cols())
{
    _Aineq = Aineq;
    _bLowerBound = bLowerBound;
    _bUpperBound = bUpperBound;

    assert( (_Aineq.rows() == _bLowerBound.rows()) &&
            (_Aineq.rows() == _bUpperBound.rows()));
}

