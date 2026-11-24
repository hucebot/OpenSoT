/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __BOUNDS_BILATERALCONSTRAINT_H__
#define __BOUNDS_BILATERALCONSTRAINT_H__

#include <OpenSoT/Constraint.h>

#include <Eigen/Dense>
#include <list>


 namespace OpenSoT {
    namespace constraints {

        /**
         * @brief The BilateralConstraint class implements a constraint of the form
         *        bLowerBound <= Aineq*x <= bUpperbound
         */
        class BilateralConstraint: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            static int _constr_count;
        public:
            typedef std::shared_ptr<BilateralConstraint> Ptr;

            /**
             * @brief BilateralConstraint a bilateral constraint
             * @param Aineq constraint matrix. Number of columns must be > 0
             * @param bLowerBound lower bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             * @param bUpperBound upper bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             */
            BilateralConstraint(const Eigen::MatrixXd &Aineq,
                                const Eigen::VectorXd &bLowerBound,
                                const Eigen::VectorXd &bUpperBound);

            /**
             * @brief BilateralConstraint a bilateral constraint
             * @param constraintName the name of the constraint
             * @param Aineq constraint matrix. Number of columns must be > 0
             * @param bLowerBound lower bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             * @param bUpperBound upper bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             */
            BilateralConstraint(const std::string constraintName,
                                const Eigen::MatrixXd &Aineq,
                                const Eigen::VectorXd &bLowerBound,
                                const Eigen::VectorXd &bUpperBound);

        protected:
            /**
             * @brief _log can be used to log internal Constraint variables
             * @param logger a shared pointer to a MatLogger
             */
            virtual void _log(XBot::MatLogger2::Ptr logger)
            {

            }

            void _update(){}

        };
    }
 }

#endif
