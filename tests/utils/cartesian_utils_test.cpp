#include <gtest/gtest.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <random>

namespace {



class testCartesianUtils: public ::testing::Test
{

protected:

    testCartesianUtils()
    {

    }

    virtual ~testCartesianUtils() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testCartesianUtils, testPseudoInverse1)
{
    Eigen::MatrixXd A(32,32);
    A.setZero();
    Eigen::MatrixXd Ainv(32,32);
    Ainv.setZero();
    Eigen::MatrixXd Apinv(32,32);
    Apinv.setZero();

    SVDPseudoInverse<Eigen::MatrixXd> pinv(A);


    for(unsigned int i = 0; i < 1000; ++i)
    {
        srand((unsigned int) time(0));
        A.setRandom();

        Ainv = A.inverse();
        pinv.compute(A, Apinv);

        for(unsigned int j = 0; j < A.rows(); ++j)
        {
            for(unsigned int k = 0; k < A.cols(); ++k)
                EXPECT_NEAR(Ainv(j,k), Apinv(j,k), 1e-8);
        }
    }

    srand((unsigned int) time(0));
    A.setRandom();
    LDLTInverse<Eigen::MatrixXd> LDLTinv(A);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        srand((unsigned int) time(0));
        A.setRandom();
        A = (A*A.transpose()).eval();

        Ainv = A.inverse();
        LDLTinv.compute(A, Apinv);

        for(unsigned int j = 0; j < A.rows(); ++j)
        {
            for(unsigned int k = 0; k < A.cols(); ++k)
                EXPECT_NEAR(Ainv(j,k), Apinv(j,k), 1e-5);
        }
    }
}


TEST_F(testCartesianUtils, testMechanum4XJacobian)
{
    double px = 0.5;
    double py = 0.3;
    double r = 0.025;

    Eigen::MatrixXd J = cartesian_utils::Mechanum4XJacobian(px, py, r);
    std::cout<<"Mechanum4XJacobian: \n"<<J<<std::endl;

    Eigen::MatrixXd H(4,3);
    H.row(0) = Eigen::Vector3d(1, -1, -px -py);
    H.row(1) = Eigen::Vector3d(1,  1,  px +py);
    H.row(2) = Eigen::Vector3d(1,  1, -px -py);
    H.row(3) = Eigen::Vector3d(1, -1,  px +py);
    H *= (1./r);
    std::cout<<"H: \n"<<H<<std::endl;

    for(unsigned int i = 0; i < J.rows(); ++i)
    {
        for(unsigned int j = 0; j < J.cols(); ++j)
            EXPECT_DOUBLE_EQ(J(i,j), H(i,j));
    }

    Eigen::MatrixXd Ji = (J.transpose() * J).inverse() * J.transpose();
    std::cout<<"Mechanum4XJacobian Inverse: \n"<<Ji<<std::endl;

    Eigen::MatrixXd Hi(3,4);
    Hi.col(0) = Eigen::Vector3d(1, -1, -1./(px + py));
    Hi.col(1) = Eigen::Vector3d(1,  1,  1./(px + py));
    Hi.col(2) = Eigen::Vector3d(1,  1, -1./(px + py));
    Hi.col(3) = Eigen::Vector3d(1, -1,  1./(px + py));
    Hi *= (r/4.);
    std::cout<<"Hi: \n"<<Hi<<std::endl;

    for(unsigned int i = 0; i < Ji.rows(); ++i)
    {
        for(unsigned int j = 0; j < Ji.cols(); ++j)
            EXPECT_DOUBLE_EQ(Ji(i,j), Hi(i,j));
    }
}




}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
