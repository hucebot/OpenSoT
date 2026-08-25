#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/solvers/iHQP.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <xbot2_interface/collision.h>
#include <chrono>
#include <OpenSoT/utils/AutoStack.h>
#include <fstream>
#include <OpenSoT/utils/resources_utils.h>

#include <urdf_parser/urdf_parser.h>

#define STATIC_POINTER_CAST std::static_pointer_cast
#define DYNAMIC_POINTER_CAST std::dynamic_pointer_cast
#define SHARED_PTR std::shared_ptr
#define MAKE_SHARED std::make_shared

namespace {
class testCollisionAvoidanceConstraint : public ::testing::Test
{

public:
  testCollisionAvoidanceConstraint()
  {

      std::string urdf_capsule_path = OpenSoT::resources_utils::find("bigman_capsules.rviz")->string();
      // TODO XXX what's this ? not used ?
      std::ifstream f(urdf_capsule_path);
      std::stringstream ss;
      ss << f.rdbuf();

      // urdf = std::make_shared<urdf::ModelInterface>();
      // urdf->initFile(urdf_capsule_path);
      std::string xml_string = ss.str();
      urdf = urdf::parseURDF(xml_string);
      // TODO XXX error checking

      std::string srdf_capsule_path = OpenSoT::resources_utils::find("bigman.srdf")->string();
      srdf = std::make_shared<srdf::Model>();
      srdf->initFile(*urdf, srdf_capsule_path);



      _model_ptr = XBot::ModelInterface::getModel(OpenSoT::resources_utils::ReadFile(urdf_capsule_path), OpenSoT::resources_utils::ReadFile(srdf_capsule_path), "pin");

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

      q = _model_ptr->getNeutralQ();
      _model_ptr->setJointPosition(q);
      _model_ptr->update();
  }

  virtual ~testCollisionAvoidanceConstraint() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  XBot::ModelInterface::Ptr _model_ptr;
  Eigen::VectorXd q;

  urdf::ModelInterfaceSharedPtr urdf;
  srdf::ModelSharedPtr srdf;

};

Eigen::VectorXd getGoodInitialPosition(const XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q = _model_ptr->getNeutralQ();
    _q[_model_ptr->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LShYaw")] = -15.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RShYaw")] = 15.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;
}

TEST_F(testCollisionAvoidanceConstraint, testEnvironmentCollisionAvoidance){

    q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();


    string base_link = "torso";
    string left_arm_link = "LSoftHandLink";
    auto left_arm_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>
                             ( base_link + "_TO_" + left_arm_link,
                               *_model_ptr,
                               left_arm_link,
                               base_link
                             );
    Eigen::Affine3d left_arm_initial_pose;
    _model_ptr->getPose ( left_arm_link, base_link, left_arm_initial_pose );
    std::cout<<"left_arm_initial_pose: "<<left_arm_initial_pose.matrix()<<std::endl;

    string right_arm_link = "RSoftHandLink";
    auto right_arm_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>
                             ( base_link + "_TO_" + right_arm_link,
                               *_model_ptr,
                               right_arm_link,
                               base_link
                             );
    Eigen::Affine3d right_arm_initial_pose;
    _model_ptr->getPose ( right_arm_link, base_link, right_arm_initial_pose );
    std::cout<<"right_arm_initial_pose: "<<right_arm_initial_pose.matrix()<<std::endl;

    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits ( q_min, q_max );
    auto joint_limit_constraint = std::make_shared<OpenSoT::constraints::velocity::JointLimits> ( *_model_ptr, q_max, q_min );

    auto joint_velocity_limit_constraint = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits> ( *_model_ptr, 1., 0.005);


    OpenSoT::constraints::velocity::CollisionAvoidance::Ptr environment_collsion_constraint =
            std::make_shared<OpenSoT::constraints::velocity::CollisionAvoidance> (
                *_model_ptr, -1, this->urdf, this->srdf);


    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == environment_collsion_constraint->getCollisionJacobian().rows());
    unsigned int max_pairs = environment_collsion_constraint->getAineq().rows();

    // we consider only environment collision avoidance
    environment_collsion_constraint->setCollisionList(std::set<std::pair<std::string, std::string>>());
    environment_collsion_constraint->update();
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getCollisionJacobian().rows() == 0);

    Eigen::Affine3d w_T_c; w_T_c.setIdentity();
    w_T_c.translation()<< 0.7, 0, 0.;
    XBot::Collision::Shape::Box box;
    box.size<<0.1, 0.6, 1.4;

    EXPECT_TRUE(environment_collsion_constraint->addCollisionShape("mybox", "world", box, w_T_c));
    environment_collsion_constraint->update();
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);

    std::set<std::string> interested_links = {"LShp","LShr","LShy","LElb","LForearm","LSoftHandLink"};

    environment_collsion_constraint->setLinksVsEnvironment(interested_links);
    environment_collsion_constraint->update();
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getCollisionJacobian().rows() == interested_links.size())<<"environment_collsion_constraint->getCollisionJacobian().rows(): "<<
                                                                                                            environment_collsion_constraint->getCollisionJacobian().rows()<<" WHILE "<<
                                                                                                           "interested_links.size(): "<<interested_links.size()<<std::endl;

    max_pairs = 300;
    environment_collsion_constraint->setMaxPairs(max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getCollisionJacobian().rows() == interested_links.size());



    environment_collsion_constraint->setDetectionThreshold(1.);
    environment_collsion_constraint->setLinkPairThreshold(0.0001);
    environment_collsion_constraint->setBoundScaling(1.);





    auto autostack_ = std::make_shared<OpenSoT::AutoStack> ( left_arm_task + right_arm_task); // + 0.2*postural_task%indices
    autostack_ << joint_limit_constraint;
    autostack_ << environment_collsion_constraint;
    autostack_<<joint_velocity_limit_constraint;

    /* Create solver */
   double eps_regularization = 1e6;
   OpenSoT::solvers::solver_back_ends solver_backend = OpenSoT::solvers::solver_back_ends::qpOASES;
    auto solver = std::make_shared<OpenSoT::solvers::iHQP> ( autostack_->getStack(),
                     autostack_->getBounds(),
                     eps_regularization,
                     solver_backend );




    double dt = 0.005; //[s]
    double T = 5; //[s]

    Eigen::VectorXd dq;
    dq.setZero(_model_ptr->getNv());
    for(unsigned int i = 0; i <= int(T/dt); ++i)
    {

        double t = i*dt;

        _model_ptr->setJointPosition ( q );
        _model_ptr->update();

        double length = 0.2;
        double period = 3;
        Eigen::Affine3d desired_pose;
        desired_pose.linear() = left_arm_initial_pose.linear();
        desired_pose.translation() = left_arm_initial_pose.translation() + Eigen::Vector3d ( 1,0,1 ) *0.3*length* ( 1-std::cos ( 2*3.1415/period*t ) );
        left_arm_task->setReference ( desired_pose.matrix() );

        desired_pose.linear() = right_arm_initial_pose.linear();
        desired_pose.translation() = right_arm_initial_pose.translation() + Eigen::Vector3d ( 1,0,1 ) *0.3*length* ( 1-std::cos ( 2*3.1415/period*t ) );
        right_arm_task->setReference ( desired_pose.matrix() );


        autostack_->update ();
        EXPECT_TRUE(solver->solve ( dq ));
        q = _model_ptr->sum(q, dq);


    }

    /**
     * Due to environment the final y position of the left arm should be < than the final position of the right arm
     */
    Eigen::Affine3d w_T_torso;
    _model_ptr->getPose("torso", w_T_torso);
    Eigen::Affine3d w_T_la = w_T_torso*Eigen::Affine3d(left_arm_task->getActualPose());
    Eigen::Affine3d w_T_ra = w_T_torso*Eigen::Affine3d(right_arm_task->getActualPose());

    EXPECT_NEAR(std::fabs(w_T_la.translation()[0] - w_T_ra.translation()[0]), 0.0970416, 1e-7); //checked empirically...
    std::cout<<"std::fabs(w_T_la.translation()[0] - w_T_ra.translation()[0]): "<<std::fabs(w_T_la.translation()[0] - w_T_ra.translation()[0])<<std::endl;
    std::cout<<"w_T_la.translation()[0]: "<<w_T_la.translation()[0]<<std::endl;
    std::cout<<"w_T_ra.translation()[0]: "<<w_T_ra.translation()[0]<<std::endl;




}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
