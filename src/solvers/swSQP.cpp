#include <OpenSoT/solvers/swSQP.h>

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
        if (k==0)
        {
            int nx0 =  _ocp->stage(k)->dx->getM().rows();
            _Q[k] += 1e3 * Eigen::MatrixXd::Identity(nx0, nx0);
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

    _x0_candidate = x0;
    _u0_candidate = u0;

    _x0 = x0;
    _u0 = u0;

    _ocp->update(_x0_candidate, _u0_candidate);

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

    _dx0.setZero();

    // relinarize qp
    linearize();
    for(unsigned int iter = 1; (iter <= _opt.max_iters && (_opt.wall_time ==-1 || ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-_stats._start)).count()<_opt.wall_time) ); ++iter)
    {
        _stats.iters = iter;
        _stats.alpha = 1.;
        _stats.line_search_iters = 1;
        _stats.line_search_accepted = false;

        _stats._iter_start = std::chrono::high_resolution_clock::now();

        // solve
        if (!_qp_solver->solve(_dx0))
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
            std::cout<<"_sigma: "<<_sigma<<std::endl;

            // relinarize qp
            linearize();

            if(_sigma > _opt.max_hessian_regularization)
            {
                std::cout<< "line search failed"<< std::endl; //to better define!
                return false;
            }
        }
        else
        {
            _sigma = _opt.initial_hessian_regularization;

            _x0 = _x0_candidate;
            _u0 = _u0_candidate;

            // relinarize qp
            linearize();

            // check break criteria on QP solution
            if (convergence_criteria())
            {
                std::chrono::duration<double> iter_elapsed = std::chrono::high_resolution_clock::now() - _stats._iter_start;
                _stats.iter_time = iter_elapsed.count();
                break;
            }

            _prev_cost = _ocp->cost();
            _prev_defect = _ocp->dynamics_defect();
            _prev_viol = _ocp->constraint_violation();

            update_statistics();
            std::chrono::duration<double> iter_elapsed = std::chrono::high_resolution_clock::now() - _stats._iter_start;
            _stats.iter_time = iter_elapsed.count();
            if(_opt.verbose)
            {
                std::cout<<_stats.toOSS(_opt.verbose).str()<<"\n"<<std::endl;
            }
        }

    }

    update_statistics();
    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - _stats._start;
    _stats.total_time = elapsed.count();
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
    for (uint i = 0; i < _ocp->getNumberOfNodes(); i++)
    {
        _stats.stages_statistics[i].cost = _ocp->stage(i)->stage_cost();
        _stats.stages_statistics[i].constraint_violation = _ocp->stage(i)->stage_constraint_violation();
    }
    

    std::chrono::duration<double> iter_elapsed = std::chrono::high_resolution_clock::now() - _stats._iter_start;
    _stats.iter_time = iter_elapsed.count();
    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - _stats._start;
    _stats.total_time = elapsed.count();

}


void swSQP::init()
{
    _sigma = _opt.initial_hessian_regularization;

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
    // Bound ALL state variables (fix initial condition: dx0 = 0)
    std::vector<int> idxbx0;
    int nx0 = _ocp->stage(0)->dx->getM().rows();
    idxbx0.resize(nx0-6);
    for(int i = 6; i < nx0; ++i)
        idxbx0[i-6] = i;

    Eigen::VectorXd lbx0 = Eigen::VectorXd::Zero(nx0-6);
    Eigen::VectorXd ubx0 = Eigen::VectorXd::Zero(nx0-6);
    _qp_solver->setBoundsX(0, idxbx0, lbx0, ubx0);

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


