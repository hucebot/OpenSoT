#ifndef __OPENSOT_OC_H__
#define __OPENSOT_OC_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/utils/LieGroupsUtils.h>
#include <OpenSoT/oc/Manifolds.h>


namespace OpenSoT {


class ocp{
    public:
        typedef std::shared_ptr<ocp> Ptr;

        struct Stage{
            typedef std::shared_ptr<Stage> Ptr;

            Stage(){}

            bool isFinalStage()
            {
                if(dynamics_derivative)
                    return false;
                return true;
            }

            void update(const Eigen::VectorXd& x0, const Eigen::VectorXd& u0)
            {
                _w0.resize(x->getInputSize());
                _w0.setZero();
                _w0.head(x0.size()) = x0;
                _w0.tail(u0.size()) = u0;


                //1 update model
                model->setJointPosition(q->getValue(_w0));
                model->setJointVelocity(v->getValue(_w0));
                if(a)
                    model->setJointAcceleration(a->getValue(_w0));
                model->update();

                //2 update and evaluate state variables
                x->update();
                x->getValue(_w0);

                if (xdot){
                    xdot->update();
                    xdot->getValue(_w0);
                }
                

                //3 update and evaluate control variables (may depends on model)
                if(u)
                {
                    u->update();
                    u->getValue(_w0);
                };

                //4 update and evaluate variables
                for(unsigned int i = 0; i < variables.size(); ++i)
                {
                    variables[i]->update();
                    variables[i]->getValue(_w0);
                }

                //3 update dynamics_derivative
                if(dynamics_derivative)
                    dynamics_derivative->update();

                //4 update stack
                if(stack)
                    stack->update();

            }

            double stage_cost()
            {
                double cost = 0.;
                if(this->stack)
                {
                    cost = 0.5 * (stack->getStack()[0]->getb().transpose() * stack->getStack()[0]->getWb())[0];
                }
                return cost;
            }

            const Eigen::VectorXd& stage_dcost_dw()
            {
                _der.setZero(dx->getInputSize());
                if(stack)
                {
                    _der = ((-1.0 * stack->getStack()[0]->getA().transpose() * stack->getStack()[0]->getWb()).transpose());
                }

                return _der;
            }

            double stage_constraint_violation()
            {
                double inf_norm =0.;
                if(stack->getBounds()->getAineq().rows() > 0) //there are constraints
                {

                    _lviolations = ((stack->getBounds()->getbLowerBound()).cwiseMax(0.0));
                    _uviolations = ((-stack->getBounds()->getbUpperBound()).cwiseMax(0.0));

                    inf_norm = std::max(_uviolations.maxCoeff(), _lviolations.maxCoeff());
                }

                return inf_norm;
            }

            const Eigen::VectorXd& stage_dviolation_dw(double beta = 10.0)
            {
                _stage_dviolation_dw.setZero(dx->getInputSize());
                
                if(stack->getBounds()->getAineq().rows() == 0)
                    return _stage_dviolation_dw;
                
                _lviolations = ((stack->getBounds()->getbLowerBound()).cwiseMax(0.0));
                _uviolations = ((-stack->getBounds()->getbUpperBound()).cwiseMax(0.0));
                
                // Combine all violations
                _all_violations.resize(_lviolations.size() + _uviolations.size());
                _all_indices.resize(_lviolations.size() + _uviolations.size());
                _all_types.resize(_lviolations.size() + _uviolations.size());
                
                for(int i = 0; i < _lviolations.size(); ++i) {
                    _all_violations[i] = _lviolations(i);
                    _all_indices[i] = i;
                    _all_types[i] = 0;
                }
                
                for(int i = 0; i < _uviolations.size(); ++i) {
                    _all_violations[_lviolations.size() + i] = _uviolations(i);
                    _all_indices[_lviolations.size() + i] = i;
                    _all_types[_lviolations.size() + i] = 1;
                }
                
                // Softmax weights: w_i = exp(beta * v_i) / sum_j(exp(beta * v_j))
                double max_v = *std::max_element(_all_violations.begin(), _all_violations.end());
                
                _all_exp_vals.resize(_all_violations.size());
                double sum_exp = 0.0;
                
                for(size_t i = 0; i < _all_violations.size(); ++i) {
                    _all_exp_vals[i] = std::exp(beta * (_all_violations[i] - max_v)); // avoid overflow
                    sum_exp += _all_exp_vals[i];
                }
                
                // Compute weighted gradient
                for(size_t i = 0; i < _all_violations.size(); ++i) {
                    double weight = _all_exp_vals[i] / sum_exp;
                    
                    int idx = _all_indices[i];
                    int type = _all_types[i];
                    
                    const auto& constraint_gradient = stack->getBounds()->getAineq().row(idx);
                    
                    int sign = (type == 0) ? -1 : 1;
                    
                    _stage_dviolation_dw += weight * sign * constraint_gradient;
                    
                }
                return _stage_dviolation_dw;
            }

            double stage_dynamics_defect()
            {
                double inf_norm = 0.;
                if(dynamics_derivative)
                {
                    inf_norm = dynamics_derivative->getb().cwiseAbs().maxCoeff();
                }
                return inf_norm;

            }

            const Eigen::VectorXd& stage_ddefect_dw(double beta = 10.0)
            {
                _stage_ddefect_dw.setZero(dx->getInputSize());
                
                if(!dynamics_derivative || dynamics_derivative->getb().size() == 0)
                    return _stage_ddefect_dw;
                
                _abs_b = dynamics_derivative->getb().cwiseAbs();
                
                // Find the index of maximum absolute violation
                int max_idx;
                double max_val = _abs_b.maxCoeff(&max_idx);
                
                if(max_val == 0.0)
                    return _stage_ddefect_dw;
                
                // For smooth approximation using softmax
                _violations.resize(_abs_b.size());
                _indices.resize(_abs_b.size());
                _signs.resize(_abs_b.size());
                
                for(int i = 0; i < _abs_b.size(); ++i) {
                    _violations[i] = _abs_b(i);
                    _indices[i] = i;
                    _signs[i] = (dynamics_derivative->getb()(i) >= 0) ? 1 : -1;
                }
                
                // Softmax weights
                double max_v = *std::max_element(_violations.begin(), _violations.end());
                
                _exp_vals.resize(_violations.size());
                double sum_exp = 0.0;
                
                for(size_t i = 0; i < _violations.size(); ++i) {
                    _exp_vals[i] = std::exp(beta * (_violations[i] - max_v));
                    sum_exp += _exp_vals[i];
                }
                
                // Compute weighted gradient
                for(size_t i = 0; i < _violations.size(); ++i) {
                    double weight = _exp_vals[i] / sum_exp;
                    int idx = _indices[i];
                    int sign = _signs[i];
                    
                    _stage_ddefect_dw += weight * sign * dynamics_derivative->getA().row(idx);
                }
                
                return _stage_ddefect_dw;
            }

            std::shared_ptr<XBot::ModelInterface> model;
            std::vector<std::shared_ptr<VariableXd>> variables;
            tasks::Aggregated::TaskPtr dynamics_derivative;
            std::shared_ptr<VariableXd> x, xdot, u, q, v, a, dx, du;
            AutoStack::Ptr stack;
            Space::Ptr state_space;

            private:
                Eigen::VectorXd _w0, _dw0;

                Eigen::VectorXd _der;

                Eigen::VectorXd _abs_b, _stage_ddefect_dw;
                std::vector<double> _violations, _exp_vals;
                std::vector<int> _indices, _signs;

                Eigen::VectorXd _stage_dviolation_dw;
                std::vector<double> _all_violations, _all_exp_vals;
                std::vector<int> _all_indices, _all_types;

                Eigen::VectorXd _lviolations, _uviolations;
        };

        typedef std::vector<Stage::Ptr> horizon;


        ocp();

        /**
         * @brief cost compute cumulative costs for all stages
         * @return cumulative cost
         */
        double cost();
        double dynamics_defect();
        double constraint_violation();

        void addStage(Stage::Ptr stage);

        void update(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0);

        Stage::Ptr stage(const unsigned int i){return _stages[i];}
        horizon& getHorizon(){return _stages;}

        unsigned int getNumberOfNodes();


    private:
        horizon _stages;

};

}

#endif
