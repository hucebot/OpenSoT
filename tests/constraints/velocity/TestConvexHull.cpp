#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <cmath>
#include <xbot2_interface/xbotinterface2.h>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s
#define toRad(X) (X * M_PI/180.0)
#include "../../common.h"

using namespace OpenSoT::constraints::velocity;


namespace {

// The fixture for testing class ConvexHull.
class testConvexHull : public TestBase{
protected:


  // You can remove any or all of the following functions if its body
  // is empty.

  testConvexHull(): TestBase("coman_floating_base.urdf")
  {
      // You can do set-up work for each test here.

      _links_in_contact.push_back("l_foot_lower_left_link");
      _links_in_contact.push_back("l_foot_lower_right_link");
      _links_in_contact.push_back("l_foot_upper_left_link");
      _links_in_contact.push_back("l_foot_upper_right_link");
      _links_in_contact.push_back("r_foot_lower_left_link");
      _links_in_contact.push_back("r_foot_lower_right_link");
      _links_in_contact.push_back("r_foot_upper_left_link");
      _links_in_contact.push_back("r_foot_upper_right_link");

      velocityLimits.setZero(3); velocityLimits<<CoMVelocityLimit,CoMVelocityLimit,CoMVelocityLimit;
      _convexHull = new ConvexHull(*(_model_ptr.get()), _links_in_contact);
  }

  virtual ~testConvexHull() {
    // You can do clean-up work that doesn't throw exceptions here.
      if(_convexHull != NULL) {
        delete _convexHull;
        _convexHull = NULL;
      }
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      _convexHull->update();
      _model_ptr->setJointPosition(_model_ptr->getNeutralQ());
      _model_ptr->update();
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for ConvexHull.

  OpenSoT::constraints::velocity::ConvexHull* _convexHull;

  Eigen::VectorXd velocityLimits;
  Eigen::VectorXd q;

  std::list<std::string> _links_in_contact;
};

void updateModel(const Eigen::VectorXd& q, XBot::ModelInterface::Ptr model)
{
    model->setJointPosition(q);
    model->update();
}



TEST_F(testConvexHull, sizesAreCorrect) {

    EXPECT_EQ(0, _convexHull->getLowerBound().size()) << "lowerBound should have size 0"
                                                      << "but has size"
                                                      <<  _convexHull->getLowerBound().size();
    EXPECT_EQ(0, _convexHull->getUpperBound().size()) << "upperBound should have size 0"
                                                      << "but has size"
                                                      << _convexHull->getUpperBound().size();



    EXPECT_EQ(_model_ptr->getNv(),_convexHull->getAineq().cols()) <<  " Aineq should have number of columns equal to "
                                                                              << _model_ptr->getNv()
                                                                              << " but has has "
                                                                              << _convexHull->getAineq().cols()
                                                                              << " columns instead";

    EXPECT_EQ(_links_in_contact.size(),_convexHull->getbLowerBound().size()) << "beq should have size 3"
                                                      << "but has size"
                                                      << _convexHull->getbLowerBound().size();




    EXPECT_EQ(_links_in_contact.size(),_convexHull->getAineq().rows()) << "Aineq should have size "
                                                       << _links_in_contact.size()
                                                       << " but has size"
                                                       << _convexHull->getAineq().rows();


    EXPECT_EQ(_links_in_contact.size(),_convexHull->getbUpperBound().size()) << "beq should have size "
                                                             << _links_in_contact.size()
                                                             << " but has size"
                                                             << _convexHull->getbUpperBound().size();
}


TEST_F(testConvexHull, NoZeroRowsPreset) {
    Eigen::VectorXd q(_model_ptr->getNq());
    q<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -0.431797,	 0.005336,	 0.000954,	 0.878479,	-0.000438,	-0.417689,	-0.435283,	-0.000493,	 0.000097,	 0.873527,	-0.000018,	-0.436310,	 0.000606,	-0.002125,	 0.000050,	 0.349666,	 0.174536,	 0.000010,	-1.396576,	-0.000000,	-0.000029,	-0.000000,	 0.349665,	-0.174895,	-0.000196,	-1.396547,	-0.000000,	-0.000026,	-0.000013;

    // TODO implement a test that checks, for this specific configuration,
    // that the solution for the convex null does not contain a row full of zeroes
    for(unsigned int i = 0; i < 10000; ++i)
    {
        if(i>1){
            q = _model_ptr->generateRandomQ();
            q.head(7)<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1;}

        _model_ptr->setJointPosition(q);
        _model_ptr->update();
        _convexHull->update();
        std::vector<Eigen::Vector3d> ch;
        _convexHull->getConvexHull(ch);
        Eigen::MatrixXd A_ch(_links_in_contact.size(),2);
        Eigen::VectorXd b_ch(_links_in_contact.size());
        _convexHull->getConstraints(ch,A_ch,b_ch,0.01);
        for(unsigned int i = 0; i < ch.size(); ++i)
            EXPECT_GT(A_ch.row(i).norm(),1E-5);
    }
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testConvexHull, BoundsAreCorrect) {

    // ------- Set The robot in a certain configuration ---------
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    q[_model_ptr->getQIndex("LHipSag")] = toRad(-23.5);
    q[_model_ptr->getQIndex("LHipLat")] = toRad(2.0);
    q[_model_ptr->getQIndex("LHipYaw")] = toRad(-4.0);
    q[_model_ptr->getQIndex("LKneeSag")] = toRad(50.1);
    q[_model_ptr->getQIndex("LAnkLat")] = toRad(-2.0);
    q[_model_ptr->getQIndex("LAnkSag")] = toRad(-26.6);

    q[_model_ptr->getQIndex("RHipSag")] = toRad(-23.5);
    q[_model_ptr->getQIndex("RHipLat")] = toRad(-2.0);
    q[_model_ptr->getQIndex("RHipYaw")] = toRad(0.0);
    q[_model_ptr->getQIndex("RKneeSag")] = toRad(50.1);
    q[_model_ptr->getQIndex("RAnkLat")] = toRad(2.0);
    q[_model_ptr->getQIndex("RAnkSag")] = toRad(-26.6);


    updateModel(q, _model_ptr);
    _convexHull->update();


    //Compute CH from internal
    std::vector<Eigen::Vector3d> ch;
    _convexHull->getConvexHull(ch);

    std::cout << "CH:"<<std::endl;
    for(unsigned int i = 0; i < ch.size(); ++i)
        std::cout << ch[i].x() << " " << ch[i].y() << std::endl;


    ASSERT_EQ(ch.size(), 6);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
