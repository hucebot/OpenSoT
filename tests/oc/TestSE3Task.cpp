// #include <pinocchio/multibody/model.hpp>
// #include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <OpenSoT/oc/SE3Task.h>
#include <Eigen/Dense>
#include <OpenSoT/utils/LieGroupsUtils.h>

#include <gtest/gtest.h>
#include <chrono>
#include <fstream>


std::string ReadFile(std::string path)
{
    std::ifstream t(path);
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

namespace {

class testSE3Task: public ::testing::Test,
                   public ::testing::WithParamInterface<OpenSoT::oc::SE3Task::ReferenceFrame>
{
    protected:
        testSE3Task()
        {

        }

        virtual ~testSE3Task()
        {

        }

        virtual void SetUp()
        {

        }

        virtual void TearDown()
        {

        }
};

double randomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

Eigen::Quaterniond randomQuaternion() {
    double u1 = randomDouble(0.0, 1.0);
    double u2 = randomDouble(0.0, 2.0 * M_PI);
    double u3 = randomDouble(0.0, 2.0 * M_PI);

    double sqrt1MinusU1 = std::sqrt(1 - u1);
    double sqrtU1 = std::sqrt(u1);

    double w = sqrt1MinusU1 * std::sin(u2);
    double x = sqrt1MinusU1 * std::cos(u2);
    double y = sqrtU1 * std::sin(u3);
    double z = sqrtU1 * std::cos(u3);

    return Eigen::Quaterniond(w, x, y, z).normalized();
}

Eigen::Affine3d generateRandomAffine3d(double min, double max) {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    // Random translation
    Eigen::Vector3d p;
    for (int i = 0; i < 3; ++i) {
        p[i] = randomDouble(min, max);
    }
    T.translation() = p;

    // Random rotation (same quaternion generator you use)
    Eigen::Quaterniond q = randomQuaternion();
    T.linear() = q.toRotationMatrix();

    return T;
}

Eigen::VectorXd generateRandomPose(double min, double max) {
    Eigen::VectorXd pose(7);
    pose.setZero();

    // Random position
    for (int i = 0; i < 3; ++i) {
        pose[i] = randomDouble(min, max);
    }

    // Random orientation (convert quaternion to Euler angles)
    Eigen::Quaterniond q = randomQuaternion();
    
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();
    pose[6] = q.w();

    return pose;
}

Eigen::VectorXd generateRandomConfig(const Eigen::VectorXd& qmin, const Eigen::VectorXd& qmax)
{
    Eigen::VectorXd q(qmin.size());
    q.setZero();
    for(unsigned int i = 0; i < qmin.size(); ++i)
    {
        q[i] = randomDouble(qmin[i], qmax[i]);
    }
    return q;
}


Eigen::MatrixXd computeFiniteDifferenceJacobian(XBot::ModelInterface::Ptr _robot, OpenSoT::oc::SE3Task::Ptr SE3T,
                                                const Eigen::VectorXd& q,  const double eps=1e-6)
{
    auto dq = Eigen::VectorXd(_robot->getNv());
    auto _q = Eigen::VectorXd(_robot->getNq());
    auto Jdiff = Eigen::MatrixXd(6, _robot->getNv());
    Jdiff.setZero();
    for(unsigned int i = 0; i < _robot->getNv(); ++i)
    {
         dq.setZero();
         dq[i] = eps;
         _q = _robot->sum(q, dq);

         _robot->setJointPosition(_q);
         _robot->update();
         SE3T->update();

         auto bplus = SE3T->getb();

         _q = _robot->sum(q, -dq);
         _robot->setJointPosition(_q);
         _robot->update();
         SE3T->update();

         auto bminus = SE3T->getb();

         Jdiff.col(i) = (bplus - bminus)/(2.*eps);
     }

    return Jdiff;
}

inline void adjoint(const Eigen::Affine3d& T, Eigen::Matrix6d& Adj) {
    Adj.setZero();

    Adj.setZero();
    Adj.topLeftCorner<3,3>() = T.linear();
    Adj.topRightCorner<3,3>() = OpenSoT::hat(T.translation()) * T.linear();
    Adj.bottomRightCorner<3,3>() =  T.linear();
}

TEST_P(testSE3Task, testJacobianFloatingFrame)
{
    OpenSoT::oc::SE3Task::ReferenceFrame reference_frame = GetParam();

    std::string path_to_urdf = OPENSOT_TEST_PATH;
    path_to_urdf += "/robots/floating_frame/floating_frame.urdf";
    std::string frame_name = "base_link";


    pinocchio::Model model;
    pinocchio::urdf::buildModel(path_to_urdf, model);
    pinocchio::Data data(model);

    XBot::ModelInterface::Ptr _robot = XBot::ModelInterface::getModel(ReadFile(path_to_urdf), OPENSOT_TEST_MODEL_TYPE);
    
    Eigen::VectorXd q = pinocchio::neutral(model);

    _robot->setJointPosition(q);
    _robot->update();

    std::vector<std::pair<std::string, int>> var_list;
    
    var_list.emplace_back("dq", _robot->getNv());
    
    OpenSoT::OptvarHelper var(var_list);

    OpenSoT::oc::SE3Task::Ptr SE3T = std::make_shared<OpenSoT::oc::SE3Task>(OpenSoT::oc::SE3Task("SE3T", *_robot, var.getVariable("dq"), frame_name, reference_frame));
    SE3T->update();

    Eigen::MatrixXd J_pin(6, model.nv);
    pinocchio::FrameIndex frame_id = model.getFrameId(frame_name);
    Eigen::Affine3d Tref;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        Tref = generateRandomAffine3d(-2., 2.);

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        if(reference_frame == OpenSoT::oc::SE3Task::ReferenceFrame::LOCAL)
            pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, J_pin);
        else
            pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::WORLD, J_pin);

        _robot->setJointPosition(q);
        _robot->update();

        SE3T->setReference(Tref);
        SE3T->update();


        ASSERT_EQ(J_pin.rows(), SE3T->getA().rows()) << "Jacobian row counts differ";
        ASSERT_EQ(J_pin.cols(), SE3T->getA().cols()) << "Jacobian column counts differ";
        
        std::cout<<"q: \n"<<q.transpose()<<std::endl;
        std::cout<<"Tref: \n"<<Tref.matrix()<<std::endl;
        std::cout<<"J_pin: \n"<<J_pin<<std::endl;
        std::cout<<"SE3T->getFrameJacobian(): \n"<<SE3T->getFrameJacobian()<<std::endl;

        // Use approximate equality for floating point comparison
        double tolerance = 1e-12;
        ASSERT_TRUE(J_pin.isApprox(SE3T->getFrameJacobian(), tolerance))
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nPinocchio Jacobian:\n" << J_pin
            << "\nOpenSoT Jacobian:\n" << SE3T->getFrameJacobian()
            << "\nDifference:\n" << (J_pin - SE3T->getFrameJacobian());

        std::cout<<"SE3T->getA(): \n"<<SE3T->getA()<<std::endl;

        Eigen::Matrix6d Jlog6;
        pinocchio::SE3Tpl<double, 0> M;
        M.translation() = SE3T->getSE3Error().inverse().translation();
        M.rotation() = SE3T->getSE3Error().inverse().linear();
        pinocchio::Jlog6(M, Jlog6);
        Eigen::MatrixXd Jp;
        if(reference_frame == OpenSoT::oc::SE3Task::ReferenceFrame::LOCAL)
            Jp = -Jlog6 * SE3T->getFrameJacobian();
        else
        {
            pinocchio::SE3& M_world_frame = data.oMf[frame_id];
            Eigen::Affine3d T;
            T.translation() = M_world_frame.translation();
            T.linear() = M_world_frame.rotation();
            Eigen::Matrix6d Adj;
            adjoint(T.inverse(), Adj);
            Jp = -Jlog6 * Adj * SE3T->getFrameJacobian();;
        }
        std::cout<<"Jp: \n"<<Jp<<std::endl;

        tolerance = 1e-6;
        ASSERT_TRUE(SE3T->getA().isApprox(Jp, tolerance))
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nJacobian computed using Pinocchio:\n" << Jp
            << "\nOpenSoT getA:\n" << SE3T->getA()
            << "\nDifference:\n" << (Jp - SE3T->getA());

        auto Jdiff = computeFiniteDifferenceJacobian(_robot, SE3T, q, 1e-6);
        std::cout<<"Jdiff: \n"<<Jdiff<<std::endl;


        q = generateRandomPose(-3., 3.);
    }
}

TEST_P(testSE3Task, testJacobianManipulatorEndEffector)
{
    OpenSoT::oc::SE3Task::ReferenceFrame reference_frame = GetParam();

    std::string path_to_urdf = OPENSOT_TEST_PATH;
    path_to_urdf += "/robots/panda/panda.urdf";
    std::string frame_name = "fp3_link8";


    pinocchio::Model model;
    pinocchio::urdf::buildModel(path_to_urdf, model);
    pinocchio::Data data(model);

    XBot::ModelInterface::Ptr _robot = XBot::ModelInterface::getModel(ReadFile(path_to_urdf), OPENSOT_TEST_MODEL_TYPE);
    
    Eigen::VectorXd q = pinocchio::neutral(model);

    _robot->setJointPosition(q);
    _robot->update();

    std::vector<std::pair<std::string, int>> var_list;
    
    var_list.emplace_back("dq", _robot->getNv());
    
    OpenSoT::OptvarHelper var(var_list);

    OpenSoT::oc::SE3Task::Ptr SE3T = std::make_shared<OpenSoT::oc::SE3Task>(OpenSoT::oc::SE3Task("SE3T", *_robot, var.getVariable("dq"), frame_name, reference_frame));
    SE3T->update();

    Eigen::MatrixXd J_pin(6, model.nv);
    pinocchio::FrameIndex frame_id = model.getFrameId(frame_name);
    Eigen::VectorXd qmin, qmax;
    _robot->getJointLimits(qmin, qmax);
    Eigen::Affine3d Tref;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        Tref = generateRandomAffine3d(-2., 2.);

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        if(reference_frame == OpenSoT::oc::SE3Task::ReferenceFrame::LOCAL)
            pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, J_pin);
        else
            pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::WORLD, J_pin);

        _robot->setJointPosition(q);
        _robot->update();

        SE3T->setReference(Tref);
        SE3T->update();


        ASSERT_EQ(J_pin.rows(), SE3T->getA().rows()) << "Jacobian row counts differ";
        ASSERT_EQ(J_pin.cols(), SE3T->getA().cols()) << "Jacobian column counts differ";


        // Use approximate equality for floating point comparison
        double tolerance = 1e-12;
        ASSERT_TRUE(J_pin.isApprox(SE3T->getFrameJacobian(), tolerance))
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nPinocchio Jacobian:\n" << J_pin
            << "\nOpenSoT Jacobian:\n" << SE3T->getFrameJacobian()
            << "\nDifference:\n" << (J_pin - SE3T->getFrameJacobian());

        std::cout<<"SE3T->getA(): \n"<<SE3T->getA()<<std::endl;

        Eigen::Matrix6d Jlog6;
        pinocchio::SE3Tpl<double, 0> M;
        M.translation() = SE3T->getSE3Error().inverse().translation();
        M.rotation() = SE3T->getSE3Error().inverse().linear();
        pinocchio::Jlog6(M, Jlog6);
        Eigen::MatrixXd Jp;
        if(reference_frame == OpenSoT::oc::SE3Task::ReferenceFrame::LOCAL)
            Jp = -Jlog6 * SE3T->getFrameJacobian();
        else
        {
            pinocchio::SE3& M_world_frame = data.oMf[frame_id];
            Eigen::Affine3d T;
            T.translation() = M_world_frame.translation();
            T.linear() = M_world_frame.rotation();
            Eigen::Matrix6d Adj;
            adjoint(T.inverse(), Adj);
            Jp = -Jlog6 * Adj * SE3T->getFrameJacobian();;
        }
        std::cout<<"Jp: \n"<<Jp<<std::endl;

        tolerance = 1e-6;
        ASSERT_TRUE(SE3T->getA().isApprox(Jp, tolerance))
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nJacobian computed using Pinocchio:\n" << Jp
            << "\nOpenSoT getA:\n" << SE3T->getA()
            << "\nDifference:\n" << (Jp - SE3T->getA());

        auto Jdiff = computeFiniteDifferenceJacobian(_robot, SE3T, q, 1e-6);
        std::cout<<"Jdiff: \n"<<Jdiff<<std::endl;


        q = generateRandomConfig(qmin, qmax);
    }
}




Eigen::VectorXd FFgenerateRandomConfig(const Eigen::VectorXd& qmin, const Eigen::VectorXd& qmax)
{
    Eigen::VectorXd q(qmin.size());
    q.setZero();
    for(unsigned int i = 0; i < qmin.size(); ++i)
    {
        q[i] = randomDouble(qmin[i], qmax[i]);
    }
    q.segment(0,7) = generateRandomPose(-2,2);

    return q;
}

TEST_P(testSE3Task, testJacobianHumanoidBase)
{
    OpenSoT::oc::SE3Task::ReferenceFrame reference_frame = GetParam();

    std::string path_to_urdf = OPENSOT_TEST_PATH;
    path_to_urdf += "/robots/coman/coman.urdf";
    std::string frame_name = "l_sole";


    pinocchio::Model model;
    pinocchio::urdf::buildModel(path_to_urdf, model);
    pinocchio::Data data(model);

    XBot::ModelInterface::Ptr _robot = XBot::ModelInterface::getModel(ReadFile(path_to_urdf), OPENSOT_TEST_MODEL_TYPE);
    
    Eigen::VectorXd q = pinocchio::neutral(model);

    _robot->setJointPosition(q);
    _robot->update();

    std::vector<std::pair<std::string, int>> var_list;
    
    var_list.emplace_back("dq", _robot->getNv());
    
    OpenSoT::OptvarHelper var(var_list);

    OpenSoT::oc::SE3Task::Ptr SE3T = std::make_shared<OpenSoT::oc::SE3Task>(OpenSoT::oc::SE3Task("SE3T", *_robot, var.getVariable("dq"), frame_name, reference_frame));
    SE3T->update();

    Eigen::MatrixXd J_pin(6, model.nv);
    pinocchio::FrameIndex frame_id = model.getFrameId(frame_name);
    Eigen::VectorXd qmin, qmax;
    _robot->getJointLimits(qmin, qmax);
    Eigen::Affine3d Tref;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        Tref = generateRandomAffine3d(-2., 2.);

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        if(reference_frame == OpenSoT::oc::SE3Task::ReferenceFrame::LOCAL)
            pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, J_pin);
        else
            pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::WORLD, J_pin);

        _robot->setJointPosition(q);
        _robot->update();

        SE3T->setReference(Tref);
        SE3T->update();


        ASSERT_EQ(J_pin.rows(), SE3T->getA().rows()) << "Jacobian row counts differ";
        ASSERT_EQ(J_pin.cols(), SE3T->getA().cols()) << "Jacobian column counts differ";


        // Use approximate equality for floating point comparison
        double tolerance = 1e-10;
        ASSERT_TRUE(J_pin.isApprox(SE3T->getFrameJacobian(), tolerance))
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nPinocchio Jacobian.T:\n" << J_pin.transpose()
            << "\nOpenSoT Jacobian.T:\n" << SE3T->getFrameJacobian().transpose()
            << "\nDifference:\n" << (J_pin - SE3T->getFrameJacobian());

        std::cout<<"SE3T->getA(): \n"<<SE3T->getA()<<std::endl;

        Eigen::Matrix6d Jlog6;
        pinocchio::SE3Tpl<double, 0> M;
        M.translation() = SE3T->getSE3Error().inverse().translation();
        M.rotation() = SE3T->getSE3Error().inverse().linear();
        pinocchio::Jlog6(M, Jlog6);
        Eigen::MatrixXd Jp;
        if(reference_frame == OpenSoT::oc::SE3Task::ReferenceFrame::LOCAL)
            Jp = -Jlog6 * SE3T->getFrameJacobian();
        else
        {
            pinocchio::SE3& M_world_frame = data.oMf[frame_id];
            Eigen::Affine3d T;
            T.translation() = M_world_frame.translation();
            T.linear() = M_world_frame.rotation();
            Eigen::Matrix6d Adj;
            adjoint(T.inverse(), Adj);
            Jp = -Jlog6 * Adj * SE3T->getFrameJacobian();;
        }
        std::cout<<"Jp: \n"<<Jp<<std::endl;

        tolerance = 1e-6;
        ASSERT_TRUE(SE3T->getA().isApprox(Jp, tolerance))
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nJacobian computed using Pinocchio:\n" << Jp
            << "\nOpenSoT getA:\n" << SE3T->getA()
            << "\nDifference:\n" << (Jp - SE3T->getA());

        auto Jdiff = computeFiniteDifferenceJacobian(_robot, SE3T, q, 1e-6);
        std::cout<<"Jdiff: \n"<<Jdiff<<std::endl;

        q = FFgenerateRandomConfig(qmin, qmax);
    }
}



INSTANTIATE_TEST_CASE_P(tryDifferentReferenceFrames,
                        testSE3Task,
                        ::testing::Values(OpenSoT::oc::SE3Task::ReferenceFrame::LOCAL, OpenSoT::oc::SE3Task::ReferenceFrame::WORLD));


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
