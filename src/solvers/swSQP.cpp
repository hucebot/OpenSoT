#include <OpenSoT/solvers/swSQP.h>
#include <fstream>
#include <iomanip>

using namespace OpenSoT::solvers;

swSQP::swSQP(OpenSoT::ocp::Ptr ocp):
    _ocp(ocp), _stats(ocp->getNumberOfNodes())
{
    _qp_solver = std::make_shared<hpipmOC>(ocp->getNumberOfNodes());

    init();
}

void swSQP::computeDynamics(const unsigned int i, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& b)
{
    A = _ocp->stage(i)->dynamics_derivative->getA().middleCols(_ocp->stage(i)->dx->getStartIdx(), _ocp->stage(i)->dx->getM().rows());
    B = _ocp->stage(i)->dynamics_derivative->getA().middleCols(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->du->getM().rows());
    b = _ocp->stage(i)->dynamics_derivative->getb();
}

void swSQP::computeCost(const unsigned int i,
                        Eigen::MatrixXd& Q, Eigen::VectorXd& q,
                        Eigen::MatrixXd& R, Eigen::VectorXd& r,
                        Eigen::MatrixXd& S)
{
    // calculating the quadratic approximation
    _H[i].triangularView<Eigen::Upper>() = _ocp->stage(i)->stack->getStack()[0]->getA().transpose() * _ocp->stage(i)->stack->getStack()[0]->getWA();
    _H[i] = _H[i].selfadjointView<Eigen::Upper>();

    // Apply sigma opnly if more than std::numeric_limits<double>::epsilon()
    if(_sigma > std::numeric_limits<double>::epsilon())
    {
        for(unsigned int j = 0; j < _H[i].rows(); ++j)
            _H[i](j,j) += _sigma;
    }

    _g[i] = - _ocp->stage(i)->stack->getStack()[0]->getA().transpose() * _ocp->stage(i)->stack->getStack()[0]->getWb();

    // computing state and control cost matrices from the quadratic approximation
    Q = _H[i].block(_ocp->stage(i)->dx->getStartIdx(), _ocp->stage(i)->dx->getStartIdx(),
                    _ocp->stage(i)->dx->getM().rows(), _ocp->stage(i)->dx->getM().rows());
    q = _g[i].segment(_ocp->stage(i)->dx->getStartIdx(), _ocp->stage(i)->dx->getM().rows());

    if(_ocp->stage(i)->u)
    {
        R = _H[i].block(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->du->getStartIdx(),
                        _ocp->stage(i)->du->getM().rows(), _ocp->stage(i)->du->getM().rows());

        S = _H[i].block(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->dx->getStartIdx(),
                        _ocp->stage(i)->du->getM().rows(), _ocp->stage(i)->dx->getM().rows());

        r = _g[i].segment(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->du->getM().rows());
    }
}

void swSQP::computeConstraints(const unsigned int i,
                               Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::VectorXd& dl, Eigen::VectorXd& du)
{
    //Do not make sense to check bnounds since bounds in the non-linear problem are constraints
    if(_ocp->stage(i)->stack->getBounds()->getAineq().rows() > 0) //there are constraints
    {
        C = _ocp->stage(i)->stack->getBounds()->getAineq().middleCols(_ocp->stage(i)->dx->getStartIdx(), _ocp->stage(i)->dx->getM().rows());
        if(_ocp->stage(i)->u){
            D = _ocp->stage(i)->stack->getBounds()->getAineq().middleCols(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->du->getM().rows());
        }

        dl = _ocp->stage(i)->stack->getBounds()->getbLowerBound();
        du = _ocp->stage(i)->stack->getBounds()->getbUpperBound();
    }
}



void swSQP::linearize()
{
    for(unsigned int k = 0; k < _ocp->getNumberOfNodes(); ++k)
    {
        // --- Dynamics (only for k < N) ---
        if(k < _ocp->getNumberOfNodes()-1)
        {
            computeDynamics(k, _A[k], _B[k], _b[k]);
            _qp_solver->setStageDynamics(k, _A[k], _B[k], _b[k]);
        }

        // --- Cost (always) ---
        computeCost(k, _Q[k], _q[k], _R[k], _r[k], _S[k]);
        if (k==0 && _opt.optimize_first_state!=0)
        {
            int nx0 =  _ocp->stage(k)->dx->getM().rows();
            _Q[k] += _opt.optimize_first_state_cost * Eigen::MatrixXd::Identity(nx0, nx0);
        }

        _qp_solver->setFullCost(k, _R[k], _Q[k], _S[k], _r[k], _q[k]);

        // --- Constraints (always) ---
        computeConstraints(k, _C[k], _D[k], _dl[k], _du[k]);
        _qp_solver->setConstraint(k, _C[k], _D[k], _dl[k], _du[k]);
    }


}

bool swSQP::solve(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0)
{
    _stats._start = std::chrono::high_resolution_clock::now();
    _sigma = _opt.initial_hessian_regularization;

    _x0_candidate = x0;
    _u0_candidate = u0;

    _x0 = x0;
    _u0 = u0;

    _ocp->update(_x0_candidate, _u0_candidate);
    linearize();


    if (_opt.line_search_strategy!=0)
    {
        _prev_cost = _ocp->cost();
        _prev_defect = _ocp->dynamics_defect();
        _prev_viol =  _ocp->constraint_violation();
    }

    for (uint i = 0; i < _ocp->getNumberOfNodes() && _opt.line_search_strategy==1 ; i++)
    {
        //dcost_dw[i] = _ocp->stage(i)->stage_dcost_dw();
        dcost_dw[i] = _g[i];
        dviol_dw[i] = _ocp->stage(i)->stage_dviolation_dw();
        ddefect_dw[i] = _ocp->stage(i)->stage_ddefect_dw();
    }

    for(unsigned int iter = 1; (iter <= _opt.max_iters && (_opt.wall_time ==-1 || ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-_stats._start)).count()<_opt.wall_time) ); ++iter)
    {
        _stats.iters = iter;
        _stats.alpha = 1.;
        _stats.line_search_iters = 1;
        _stats.line_search_accepted = false;

        _stats._iter_start = std::chrono::high_resolution_clock::now();

        // relinarize qp
        linearize();

        // solve
        if (!_qp_solver->solve())
            std::cout<< "qp nosolve: "<<_qp_solver->solveStatus()<<std::endl;
        _stats.qp_iters = _qp_solver->get_iters();
        _qp_solution = _qp_solver->getSolution();

        // first update
        step(_stats.alpha);

        _ocp->update(_x0_candidate, _u0_candidate);
        
        //Line-search
        if(_opt.line_search_strategy == 0)
            _stats.line_search_accepted = true;

        while(_opt.line_search_strategy!=0 && _stats.alpha >= _opt.alpha_min)
        {
            if((this->*ls_function)())
            {
                _stats.line_search_accepted=true;
                _sigma = _opt.initial_hessian_regularization;
                break;
            }
            _stats.alpha /= 2.;
            _stats.line_search_iters++;
            step(_stats.alpha);
            _ocp->update(_x0_candidate, _u0_candidate);
        }

        if(!_stats.line_search_accepted)
        {
            _ocp->update(_x0, _u0); //update at previous linearization point
            _sigma *= _opt.hessian_scale_factor_up; //rise regularization

            if(_sigma > _opt.max_hessian_regularization)
            {
                std::cout<< "line search failed"<< std::endl; //to better define!
                return false;
            }
        }
        else
        {
            // _sigma = _opt.initial_hessian_regularization;

            _x0 = _x0_candidate;
            _u0 = _u0_candidate;

            // check break criteria on QP solution
            if (convergence_criteria())
                break;

            _prev_cost = _ocp->cost();
            _prev_defect = _ocp->dynamics_defect();
            _prev_viol = _ocp->constraint_violation();

        }
        update_statistics();
        if(_opt.verbose)
        {
            std::cout<<_stats.toOSS(_opt.verbose).str()<<"\n"<<std::endl;
        }
    }
    update_statistics();
    if(_opt.verbose)
    {
        std::cout<<_stats.toOSS(_opt.verbose).str()<<"\n"<<std::endl;
    }

    return true;
}

bool swSQP::convergence_criteria()
{
    // 1. Change in decision variables
    double max_dsol = -INFINITY;
    for(unsigned int i = 0; i < _ocp->getNumberOfNodes(); ++i)
        max_dsol = std::max(max_dsol, _stats.alpha * _qp_solution[i].x.cwiseAbs().maxCoeff());
    
    _stats.max_dsolution =  max_dsol;
    bool step_converged = max_dsol <= _opt.min_abs_delta_solution;
    
    // 2. Constraint violation
    bool feasible = _ocp->constraint_violation() <= _opt.min_abs_delta_solution;
    
    // 3. KKT residual (optimality condition)
    double kkt_residual = compute_kkt_residual();
    bool optimal = kkt_residual <= _opt.min_abs_delta_solution;

    // Combined criteria
    bool converged = (step_converged && feasible) || (optimal && feasible);
    

    _stats.converged = (converged ? (optimal?"Optimal Solution Found":"Local minimum Solution") : "NO");

    return converged;
}

double swSQP::compute_kkt_residual()
{
    double total_kkt_sq = 0.0;
    
    for(unsigned int i = 0; i < _ocp->getNumberOfNodes(); ++i)
    {
        // ===== STATE KKT GRADIENT =====
        Eigen::VectorXd kkt_x = _q[i];  // Objective gradient w.r.t. state (already computed!)

        // Add general constraint multipliers: C^T * (lam_ug - lam_lg)
        if(_C[i].rows() > 0)
        {
            kkt_x += _C[i].transpose() * (_qp_solution[i].lam_ug - _qp_solution[i].lam_lg);
        }
        
        // Add dynamics multipliers (costate equation)
        // Current stage dynamics: -A[i]^T * pi[i]
        if(i < _ocp->getNumberOfNodes()-1)
        {
            Eigen::VectorXd pi = _qp_solution[i].pi;
            kkt_x -= _A[i].transpose() * pi;
        }
        
        // Previous stage dynamics: +pi[i-1]
        if(i > 0)
        {
            Eigen::VectorXd pi_prev = _qp_solution[i-1].pi;
            kkt_x += pi_prev;
        }
        
        total_kkt_sq += kkt_x.squaredNorm();
        
        
        // ===== CONTROL KKT GRADIENT (only for stages with control) =====
        if(i < _ocp->getNumberOfNodes()-1)
        {
            Eigen::VectorXd kkt_u = _r[i];  // Objective gradient w.r.t. control (already computed!)

            // Add general constraint multipliers: D^T * (lam_ug - lam_lg)
            if(_D[i].rows() > 0)
            {
                kkt_u += _D[i].transpose() * (_qp_solution[i].lam_ug - _qp_solution[i].lam_lg);
            }
            
            // Add dynamics multipliers: B^T * pi
            Eigen::VectorXd pi = _qp_solution[i].pi;
            kkt_u += _B[i].transpose() * pi;
            
            total_kkt_sq += kkt_u.squaredNorm();
        }
    }
    
    return std::sqrt(total_kkt_sq);
}

void swSQP::step(double alpha)
{
    for(unsigned int k = 0; k < _x0_candidate.size(); ++k)
    {
        if(_ocp->stage(k)->state_space)
        {
            _ocp->stage(k)->state_space->plus(_x0[k], alpha*_qp_solution[k].x, _x0_candidate[k]);
        }
        if (k < _u0_candidate.size())
            _u0_candidate[k] = _u0[k] + alpha * _qp_solution[k].u;

    }
}

bool swSQP::ls_merit()
{
    double merit = _ocp->cost() + _ocp->constraint_violation() + _ocp->dynamics_defect();

    
    double merit_der = 0.;

    for(unsigned int i = 0; i < _ocp->getNumberOfNodes(); ++i)
    {

        merit_der += ((dcost_dw[i].segment(_ocp->stage(i)->dx->getStartIdx(), _ocp->stage(i)->dx->getM().rows())).transpose() * _qp_solver->getSolution()[i].x)[0];

        merit_der += ((dviol_dw[i].segment(_ocp->stage(i)->dx->getStartIdx(), _ocp->stage(i)->dx->getM().rows())).transpose() * _qp_solver->getSolution()[i].x)[0];

        merit_der += ((ddefect_dw[i].segment(_ocp->stage(i)->dx->getStartIdx(), _ocp->stage(i)->dx->getM().rows())).transpose() * _qp_solver->getSolution()[i].x)[0];

        if(i<_ocp->getNumberOfNodes()-1)
        {
            merit_der += ((dcost_dw[i].segment(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->du->getM().rows())).transpose() * _qp_solver->getSolution()[i].u)[0];

            merit_der += ((dviol_dw[i].segment(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->du->getM().rows())).transpose() * _qp_solver->getSolution()[i].u)[0];

            merit_der += ((ddefect_dw[i].segment(_ocp->stage(i)->du->getStartIdx(), _ocp->stage(i)->du->getM().rows())).transpose() * _qp_solver->getSolution()[i].u)[0];
        }
    }

    double armijo = _prev_cost + _prev_viol + _prev_defect + _opt.beta * _stats.alpha * merit_der;

    if(merit < armijo || std::fabs(merit - armijo) <= std::numeric_limits<double>::epsilon())
        return  true;

    return false;
}

bool swSQP::ls_filter()
{

    if (_ocp->cost() <  _prev_cost || std::fabs(_ocp->cost() -  _prev_cost) <=  std::numeric_limits<double>::epsilon() ||
        _ocp->constraint_violation() < _prev_viol || std::fabs(_ocp->constraint_violation() - _prev_viol) <=  std::numeric_limits<double>::epsilon() ||
        _ocp->dynamics_defect() < _prev_defect || std::fabs(_ocp->dynamics_defect() - _prev_defect)  <=  std::numeric_limits<double>::epsilon())
        return true;

    return false;
}


void swSQP::update_statistics()
{
    _stats.cost = _ocp->cost();
    _stats.constraint_violation = _ocp->constraint_violation();
    _stats.hessian_reg = _sigma;
    for (uint i = 0; i < _ocp->getNumberOfNodes(); i++)
    {
        _stats.stages_statistics[i].cost = _ocp->stage(i)->stage_cost();
        _stats.stages_statistics[i].constraint_violation = _ocp->stage(i)->stage_constraint_violation();
    }


    std::chrono::duration<double> iter_elapsed = std::chrono::high_resolution_clock::now() - _stats._iter_start;
    _stats.iter_time = iter_elapsed.count();
    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - _stats._start;
    _stats.total_time = elapsed.count();

    // Store iteration history
    if(_stats.line_search_accepted)
    {
        swSQP::iteration_statistics iter_stats;
        iter_stats.iter = _stats.iters;
        iter_stats.qp_iters = _stats.qp_iters;
        iter_stats.cost = _stats.cost;
        iter_stats.constraint_violation = _stats.constraint_violation;
        iter_stats.max_dsolution = _stats.max_dsolution;
        iter_stats.alpha = _stats.alpha;
        iter_stats.line_search_iters = _stats.line_search_iters;
        iter_stats.line_search_accepted = _stats.line_search_accepted;
        iter_stats.hessian_reg = _stats.hessian_reg;
        iter_stats.iter_time = _stats.iter_time;
        _stats.iteration_history.push_back(iter_stats);
    }
}


void swSQP::init()
{

    _stats.line_search_accepted = false;
    _stats.line_search_iters = 0;
    _stats.alpha = 1;

    dcost_dw.resize(_ocp->getNumberOfNodes());
    dviol_dw.resize(_ocp->getNumberOfNodes());
    ddefect_dw.resize(_ocp->getNumberOfNodes());


    if(_opt.line_search_strategy == 1)
        ls_function = &swSQP::ls_merit;
    if(_opt.line_search_strategy == 2)
        ls_function = &swSQP::ls_filter;


    // Create list of indices for state bounds at stage 0 (initial state)
    if (_opt.optimize_first_state!=2)
    {
        std::vector<int> idxbx0;
        Eigen::VectorXd lbx0;
        Eigen::VectorXd ubx0;
        int nx0 = _ocp->stage(0)->model->getNv();
        if (_opt.optimize_first_state == 1)
        {
            idxbx0.resize(2*(nx0-6));
            for(int i = 6; i < nx0; ++i)
            {
                idxbx0[i-6] = i;
                idxbx0[nx0+i-12] = nx0 + i;
            }
            lbx0 = Eigen::VectorXd::Zero(2*(nx0-6));
            ubx0 = Eigen::VectorXd::Zero(2*(nx0-6));
        }
        if (_opt.optimize_first_state == 0)
        {
            idxbx0.resize(2*nx0);
            for(int i = 0; i < nx0; ++i)
            {
                idxbx0[i] = i;
                idxbx0[nx0+i] = nx0+i;
            }
                
            lbx0 = Eigen::VectorXd::Zero(2*nx0);
            ubx0 = Eigen::VectorXd::Zero(2*nx0);
        }
        _qp_solver->setBoundsX(0, idxbx0, lbx0, ubx0);
    }

    for(unsigned int k = 0; k < _ocp->getNumberOfNodes(); ++k)
    {

        // --- Dynamics (only for k < N) ---
        if(k < _ocp->getNumberOfNodes()-1)
        {

            Eigen::MatrixXd A, B;
            Eigen::VectorXd b;
            computeDynamics(k, A, B, b);

            _A.push_back(A);
            _B.push_back(B);
            _b.push_back(b);

            _qp_solver->setStageDynamics(k, A, B, b);
        }

        // --- Cost (always) ---
        _H.push_back(Eigen::MatrixXd(_ocp->stage(k)->stack->getStack()[0]->getA().cols(), _ocp->stage(k)->stack->getStack()[0]->getA().cols()));
        Eigen::VectorXd g;
        _g.push_back(g);


        Eigen::MatrixXd R, Q, S;
        Eigen::VectorXd r, q;
        computeCost(k, Q, q, R, r, S);
        _Q.push_back(Q);
        _q.push_back(q);
        _R.push_back(R);
        _r.push_back(r);
        _S.push_back(S);

        _qp_solver->setFullCost(k, _R[k], _Q[k], _S[k], _r[k], _q[k]);

        // --- Constraints (always) ---
        Eigen::MatrixXd C, D;
        Eigen::VectorXd dl, du;
        computeConstraints(k, C, D, dl, du);
        _C.push_back(C);
        _D.push_back(D);
        _dl.push_back(dl);
        _du.push_back(du);

        _qp_solver->setConstraint(k, _C[k], _D[k], _dl[k], _du[k]);
   
    }
    if(_opt.verbose)
            std::cout<<"Solver inited"<<std::endl;
}

bool swSQP::exportToJSON(const std::string& filename, double dt) const
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    file << std::setprecision(16);
    file << "{\n";

    // Problem configuration
    file << "  \"problem_config\": {\n";
    file << "    \"num_nodes\": " << _ocp->getNumberOfNodes() << ",\n";
    if (dt > 0.0)
        file << "    \"dt\": " << dt << ",\n";
    file << "    \"state_dim\": " << _x0[0].size() << ",\n";
    if (!_u0.empty())
        file << "    \"control_dim\": " << _u0[0].size() << ",\n";
    else
        file << "    \"control_dim\": 0,\n";
    file << "    \"num_constraints\": " << (_C[0].rows() > 0 ? _C[0].rows() : 0) << "\n";
    file << "  },\n";

    // swSQP options
    file << "  \"swsqp_options\": {\n";
    file << "    \"max_iters\": " << _opt.max_iters << ",\n";
    file << "    \"min_abs_delta_solution\": " << _opt.min_abs_delta_solution << ",\n";
    file << "    \"verbose\": " << _opt.verbose << ",\n";
    file << "    \"alpha_min\": " << _opt.alpha_min << ",\n";
    file << "    \"beta\": " << _opt.beta << ",\n";
    file << "    \"line_search_strategy\": " << _opt.line_search_strategy << ",\n";
    file << "    \"initial_hessian_regularization\": " << _opt.initial_hessian_regularization << ",\n";
    file << "    \"hessian_scale_factor_up\": " << _opt.hessian_scale_factor_up << ",\n";
    file << "    \"max_hessian_regularization\": " << _opt.max_hessian_regularization << ",\n";
    file << "    \"wall_time\": " << _opt.wall_time << ",\n";
    file << "    \"optimize_first_state\": " << _opt.optimize_first_state << ",\n";
    file << "    \"optimize_first_state_cost\": " << _opt.optimize_first_state_cost << "\n";
    file << "  },\n";

    // HPIPM options
    auto hpipm_opts = _qp_solver->getOptions();
    file << "  \"hpipm_options\": {\n";
    file << "    \"mode\": " << static_cast<int>(hpipm_opts.mode) << ",\n";
    file << "    \"iter_max\": " << hpipm_opts.iter_max << ",\n";
    file << "    \"alpha_min\": " << hpipm_opts.alpha_min << ",\n";
    file << "    \"mu0\": " << hpipm_opts.mu0 << ",\n";
    file << "    \"tol_stat\": " << hpipm_opts.tol_stat << ",\n";
    file << "    \"tol_eq\": " << hpipm_opts.tol_eq << ",\n";
    file << "    \"tol_ineq\": " << hpipm_opts.tol_ineq << ",\n";
    file << "    \"tol_comp\": " << hpipm_opts.tol_comp << ",\n";
    file << "    \"reg_prim\": " << hpipm_opts.reg_prim << ",\n";
    file << "    \"warm_start\": " << hpipm_opts.warm_start << ",\n";
    file << "    \"pred_corr\": " << hpipm_opts.pred_corr << ",\n";
    file << "    \"ric_alg\": " << hpipm_opts.ric_alg << ",\n";
    file << "    \"split_step\": " << hpipm_opts.split_step << "\n";
    file << "  },\n";

    // Final solution summary
    file << "  \"final_solution\": {\n";
    file << "    \"converged\": \"" << _stats.converged << "\",\n";
    file << "    \"total_iterations\": " << _stats.iters << ",\n";
    file << "    \"total_time\": " << _stats.total_time << ",\n";
    file << "    \"final_cost\": " << _stats.cost << ",\n";
    file << "    \"final_constraint_violation\": " << _stats.constraint_violation << ",\n";
    file << "    \"final_max_dsolution\": " << _stats.max_dsolution << "\n";
    file << "  },\n";

    // Iteration-wise statistics
    file << "  \"iteration_statistics\": [\n";
    for (size_t i = 0; i < _stats.iteration_history.size(); ++i)
    {
        const auto& iter_stats = _stats.iteration_history[i];
        file << "    {\n";
        file << "      \"iter\": " << iter_stats.iter << ",\n";
        file << "      \"qp_iters\": " << iter_stats.qp_iters << ",\n";
        file << "      \"cost\": " << iter_stats.cost << ",\n";
        file << "      \"constraint_violation\": " << iter_stats.constraint_violation << ",\n";
        file << "      \"max_dsolution\": " << iter_stats.max_dsolution << ",\n";
        file << "      \"alpha\": " << iter_stats.alpha << ",\n";
        file << "      \"line_search_iters\": " << iter_stats.line_search_iters << ",\n";
        file << "      \"line_search_accepted\": " << (iter_stats.line_search_accepted ? "true" : "false") << ",\n";
        file << "      \"hessian_reg\": " << iter_stats.hessian_reg << ",\n";
        file << "      \"iter_time\": " << iter_stats.iter_time << "\n";
        file << "    }";
        if (i < _stats.iteration_history.size() - 1)
            file << ",";
        file << "\n";
    }
    file << "  ],\n";

    // State trajectory
    file << "  \"state_trajectory\": [\n";
    for (size_t k = 0; k < _x0.size(); ++k)
    {
        file << "    [";
        for (int i = 0; i < _x0[k].size(); ++i)
        {
            file << _x0[k](i);
            if (i < _x0[k].size() - 1)
                file << ", ";
        }
        file << "]";
        if (k < _x0.size() - 1)
            file << ",";
        file << "\n";
    }
    file << "  ],\n";

    // Control trajectory
    file << "  \"control_trajectory\": [\n";
    for (size_t k = 0; k < _u0.size(); ++k)
    {
        file << "    [";
        for (int i = 0; i < _u0[k].size(); ++i)
        {
            file << _u0[k](i);
            if (i < _u0[k].size() - 1)
                file << ", ";
        }
        file << "]";
        if (k < _u0.size() - 1)
            file << ",";
        file << "\n";
    }
    file << "  ]\n";

    file << "}\n";
    file.close();

    if(_opt.verbose)
        std::cout << "Exported solver data to: " << filename << std::endl;

    return true;
}


