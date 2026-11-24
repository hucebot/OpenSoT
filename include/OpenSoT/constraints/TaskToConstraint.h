/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef __BOUNDS_TASKTOCONSTRAINT_H__
#define __BOUNDS_TASKTOCONSTRAINT_H__

#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/Task.h>



 namespace OpenSoT {
    namespace constraints {

        /**
         * @brief The TaskToConstraint class transforms a task into an inequality constraint
         * which bounds the error in between err_lb and err_ub
         *
         * BilateralConstraint:
         *   \f$ b + err_lb <= A*x <= b + err_ub \f$
         */
        class TaskToConstraint: public BilateralConstraint {
        public:
            typedef std::shared_ptr< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> > TaskPtr;
            typedef std::shared_ptr< OpenSoT::constraints::TaskToConstraint> Ptr;

        private:
            
            TaskPtr _task;
            
            Eigen::VectorXd _err_lb, _err_ub;

        public:

            /**
             * @brief TaskToConstraint creates an equality TaskToConstraint from a task
             * Calling update on the constraint will also update the task
             * @param task
             */
            TaskToConstraint(TaskPtr task);
            
             /**
             * @brief TaskToConstraint creates a TaskToConstraint from a task
             * Calling update on the constraint will also update the task
             * @param task
             */
            TaskToConstraint(TaskPtr task, 
                             const Eigen::VectorXd& err_lb, 
                             const Eigen::VectorXd& err_ub);

            void setBounds(const Eigen::VectorXd& err_lb, const Eigen::VectorXd& err_ub)
            {
                if( ((err_ub-err_lb).array() < 0).any() ){
                    throw std::runtime_error("Some components of err_ub are smaller than err_lb!!!");
                }

                if( err_lb.size() != _task->getA().rows() || err_ub.size() != _task->getA().rows() ){
                    throw std::runtime_error("Either err_ub or err_lb has wrong dimension!!!");
                }

                _err_lb = err_lb;
                _err_ub = err_ub;
            }

        protected:

            void generateAll();

            /**
             * @brief _log can be used to log internal Constraint variables
             * @param logger a shared pointer to a MatLogger
             */
            virtual void _log(XBot::MatLogger2::Ptr logger)
            {
                _task->log(logger);
            }

            /**
             * @brief update updates the adapted task and the adapter constraint
             * @param q
             */
            void _update();
        };
    }
 }

#endif
