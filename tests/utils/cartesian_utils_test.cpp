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





}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
