#include "../../tests/common.h"
#include <string>
#include <random>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/solvers/nHQP.h>
#include <qpSWIFT/qpSWIFT.h>
#include <matlogger2/matlogger2.h>
#include <OpenSoT/utils/resources_utils.h>

#include <chrono>
using namespace std::chrono;

#define USE_SOLVER_NHQP false

int main(int argc, char **argv)
{
    std::string resource = "panda.urdf";
    auto urdf_path = OpenSoT::resources_utils::find(resource);
    if(urdf_path)
        std::cout << "Found: " << *urdf_path << std::endl;
    else
    {
        std::cout << "Resource "<<resource<<" not found\n";
        return 0;
    }

    auto urdf_string = OpenSoT::resources_utils::ReadFile(urdf_path->string());

    auto model = XBot::ModelInterface::getModel(urdf_string, "pin");

    model->print(std::cout) << " OK"<<std::endl;

    Eigen::VectorXd q = model->getNeutralQ();
    q << 0., -0.7, 0., -2.1, 0., 1.4, 0.;

    model->setJointPosition(q);
    model->update();

    std::string TCP_frame = "fp3_link8";
    Eigen::Affine3d TCP_world_pose_init;
    model->getPose(TCP_frame, TCP_world_pose_init);
    std::cout<<TCP_frame<<" pose in world: \n"<<TCP_world_pose_init.matrix()<<std::endl;

    /** Stack **/
    using namespace OpenSoT::tasks::velocity;
    auto TCP = std::make_shared<Cartesian>("TCP", *model, TCP_frame, "world");
    TCP->setLambda(0.1);

    auto postural = std::make_shared<Postural>(*model, "postural");
    postural->setLambda(0.01);

    using namespace OpenSoT::constraints::velocity;
    Eigen::VectorXd qmin, qmax;
    model->getJointLimits(qmin, qmax);
    auto joint_limits = std::make_shared<JointLimits>(*model, qmax, qmin);

    Eigen::VectorXd dqlim;
    model->getVelocityLimits(dqlim);
    double dT = 0.01;
    auto vel_limits = std::make_shared<VelocityLimits>(*model, dqlim, dT);

    auto stack = (TCP%std::list<unsigned int>({0,1,2})/(TCP%std::list<unsigned int>({3,4,5}))/postural)<<joint_limits<<vel_limits;

    /** Solver **/
    double eps = 1e9;
#if USE_SOLVER_NHQP
    auto solver = std::make_shared<OpenSoT::solvers::nHQP>(stack->getStack(), stack->getBounds(), eps, OpenSoT::solvers::solver_back_ends::qpOASES);
#else
    auto solver = std::make_shared<OpenSoT::solvers::iHQP>(*stack, eps, OpenSoT::solvers::solver_back_ends::qpOASES);
#endif

    /** IK LOOP **/
    std::atomic<bool> stop(false);

    std::thread input_thread([&](){
        std::cout << "Press ENTER to stop\n";
        std::cin.get();
        stop = true;
    });

    XBot::MatLogger2::Ptr logger = XBot::MatLogger2::MakeLogger("/tmp/panda_ik");
    logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    auto dt = std::chrono::duration<double>(dT);
    auto start = std::chrono::steady_clock::now();
    Eigen::VectorXd dq(model->getNv());
    dq.setZero();
    Eigen::Affine3d pose_ref;
    TCP->getReference(pose_ref);
    for(int i = 0; !stop; ++i)
    {
        auto now = std::chrono::steady_clock::now();

        logger->add("q", q);

        model->setJointPosition(q);
        model->update();

        double w = 2 * M_PI * std::fmod(dt.count() * i, 1.0);
        pose_ref.translation()[0] += 0.01 * sin(w);
        pose_ref.translation()[1] += 0.01 * cos(w);
        TCP->setReference(pose_ref);

        stack->update();

        if(!solver->solve(dq))
        {
            std::cout<<"Solver can not solve!"<<std::endl;
            dq.setZero();
        }

        q = model->sum(q, dq);

        if (now - start >= std::chrono::seconds(1))
        {
            std::cout << "." << std::flush;
            start += std::chrono::seconds(1);
        }

        std::this_thread::sleep_for(dt);
    }

    input_thread.join();

    std::cout<<"log saved at: "<<logger->get_filename()<<std::endl;
    std::cout<<"To replay the trajectory runs:\n \n     replay --urdf_file panda.urdf --mat_file "<<logger->get_filename()<<" --var_name q --fps "<<int(1/dT)<<std::endl;

    return 0;
}



