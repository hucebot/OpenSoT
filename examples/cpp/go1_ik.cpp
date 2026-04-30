#include <string>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/nHQP.h>
#include <OpenSoT/solvers/iHQP.h>
#include <qpSWIFT/qpSWIFT.h>
#include <matlogger2/matlogger2.h>
#include <OpenSoT/utils/resources_utils.h>
#include <thread>

#include <chrono>
using namespace std::chrono;

Eigen::Vector3d upDownOscillation(const Eigen::Vector3d& p0, double t, double period = 2.0, double amplitude = 0.1, bool reverse = false)
{
    double omega = 2.0 * M_PI / period;

    Eigen::Vector3d p = p0;
    if(reverse)
        p.z() -= amplitude * std::sin(omega * t);
    else
        p.z() += amplitude * std::sin(omega * t);

    return p;
}


int main(int argc, char **argv)
{
    std::string resource = "g1.urdf";
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
    q << 0.0, 0.0, 0.793 - 0.113, 0.0, 0.0, 0.0, 1.0, // base
        -0.6,  // left_hip_pitch_joint
        0.0,   // left_hip_roll_joint
        0.0,   // left_hip_yaw_joint
        1.2,   // left_knee_joint
        -0.6,  // left_ankle_pitch_joint
        0.0,   // left_ankle_roll_joint
        -0.6,  // right_hip_pitch_joint
        0.0,   // right_hip_roll_joint
        0.0,   // right_hip_yaw_joint
        1.2,   // right_knee_joint
        -0.6,  // right_ankle_pitch_joint
        0.0,   // right_ankle_roll_joint
        0.0,   // waist_yaw_joint
        0.0,   // waist_roll_joint
        0.0,   // waist_pitch_joint
        0.0,   // left_shoulder_pitch_joint
        0.0,   // left_shoulder_roll_joint
        0.0,   // left_shoulder_yaw_joint
        0.0,   // left_wrist_pitch_joint
        0.0,   // left_wrist_yaw_joint
        0.0,   // left_elbow_joint
        0.0,   // left_wrist_roll_joint
        0.0,   // right_shoulder_pitch_joint
        0.0,   // right_shoulder_roll_joint
        0.0,   // right_shoulder_yaw_joint
        0.0,   // right_elbow_joint
        0.0,   // right_wrist_roll_joint
        0.0,   // right_wrist_pitch_joint
        0.0;   // right_wrist_yaw_joint

    model->setJointPosition(q);
    model->update();

    /** Stack **/
    using namespace OpenSoT::tasks::velocity;

    auto contact_left = std::make_shared<Cartesian>("left_ankle_roll_link", *model, "left_ankle_roll_link", "world");
    auto contact_right = std::make_shared<Cartesian>("right_ankle_roll_link", *model, "right_ankle_roll_link", "world");

    auto com = std::make_shared<CoM>(*model);

    auto arm_left = std::make_shared<Cartesian>("left_rubber_hand", *model, "left_rubber_hand", "world");
    arm_left->setLambda(0.1);
    Eigen::Affine3d T0_left;
    arm_left->getReference(T0_left);
    auto arm_right = std::make_shared<Cartesian>("right_rubber_hand", *model, "right_rubber_hand", "world");
    arm_right->setLambda(0.1);
    Eigen::Affine3d T0_right;
    arm_right->getReference(T0_right);

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

    auto stack = ((contact_left + contact_right)/(com + 0.1*arm_left + 0.1*arm_right)/postural)<<joint_limits<<vel_limits;

    /** Solver **/
    double eps = 1e9;
    auto solver = std::make_shared<OpenSoT::solvers::iHQP>(*stack, eps, OpenSoT::solvers::solver_back_ends::qpOASES);

    /** IK LOOP **/
    std::atomic<bool> stop(false);

    std::thread input_thread([&](){
        std::cout << "Press ENTER to stop\n";
        std::cin.get();
        stop = true;
    });

    XBot::MatLogger2::Ptr logger = XBot::MatLogger2::MakeLogger("/tmp/g1_ik");
    logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    auto dt = std::chrono::duration<double>(dT);
    auto start = std::chrono::steady_clock::now();
    Eigen::VectorXd dq(model->getNv());
    dq.setZero();
    logger->add("floating_base", Eigen::VectorXd::Zero(1)); // this is used to say to visualization that the robot is floating base
    double t = 0;
    Eigen::Affine3d Tref_left;
    arm_left->getReference(Tref_left);
    Eigen::Affine3d Tref_right;
    arm_right->getReference(Tref_right);
    for(int i = 0; !stop; ++i)
    {
        auto now = std::chrono::steady_clock::now();

        logger->add("q", q);
        logger->add("qdot", dq/dT);

        Tref_left.translation() = upDownOscillation(T0_left.translation(), t, 1., 0.1);
        arm_left->setReference(Tref_left);

        Tref_right.translation() = upDownOscillation(T0_right.translation(), t, 1., 0.1, true);
        arm_right->setReference(Tref_right);

        model->setJointPosition(q);
        model->update();


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

        t += dT;
        std::this_thread::sleep_for(dt);
    }

    input_thread.join();

    std::cout<<"log saved at: "<<logger->get_filename()<<std::endl;
    std::cout<<"To replay the trajectory runs:\n \n     replay --urdf_file g1.urdf --mat_file "<<logger->get_filename()<<" --q_var_name q --v_var_name qdot --fps "<<int(1/dT)<<std::endl;

    return 0;
}





