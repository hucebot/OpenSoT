#include <OpenSoT/oc/oc.h>

using namespace OpenSoT;

ocp::ocp()
{

}

void ocp::addStage(Stage::Ptr stage)
{
    _stages.push_back(stage);
}

unsigned int ocp::getNumberOfNodes()
{
    return _stages.size();
}


/**
 * @note: this way to update does not make possible to write the dynamics constraint in terms of 'i-1' but only in terms of 'i+1'.
 * Ideally we would like that is not important the way is written. to do this we should decouple in the Stage the update of the variables with the update of the problem.
 */
void ocp::update(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0)
{
    if(x0.size() != (u0.size() + 1))
        throw std::runtime_error("x0.size() != (u0.size() + 1)");

    for (int i = static_cast<int>(_stages.size()) - 1; i >= 0; --i)
    {
        if (i == static_cast<int>(_stages.size()) - 1)
        {
            _stages[i]->update(x0[i], Eigen::VectorXd(0));
        }
        else
        {
            _stages[i]->update(x0[i], u0[i]);
        }
    }
}

void ocp::updateDVariables(const std::vector<Eigen::VectorXd>& dx0, const std::vector<Eigen::VectorXd>& du0)
{
    if(dx0.size() != (du0.size() + 1))
        throw std::runtime_error("dx0.size() != (du0.size() + 1)");

    for(unsigned int i = 0; i < du0.size(); ++i)
    {
        _stages[i]->updateDVariables(dx0[i], du0[i]);
    }
    _stages[_stages.size()-1]->updateDVariables(dx0[_stages.size()-1], Eigen::VectorXd(0));
}


double ocp::cost()
{
    double cost = 0.;
    for(unsigned int i = 0; i < _stages.size(); ++i)
        cost += _stages[i]->stage_cost();
    return cost;
}

double ocp::dynamics_defect()
{
    double defect = 0.;
    for(unsigned int i = 0; i < _stages.size(); ++i)
        defect += _stages[i]->stage_dynamics_defect();
    return defect;
}

double ocp::constraint_violation()
{
    double violation = 0.;
    for(unsigned int i = 0; i < _stages.size(); ++i)
        violation += _stages[i]->stage_constraint_violation();
    return violation;

}
