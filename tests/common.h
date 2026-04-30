#ifndef COMMON_H
#define COMMON_H

#include <gtest/gtest.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/resources_utils.h>


XBot::ModelInterface::Ptr GetTestModel(std::string name)
{
    auto urdf_path = OpenSoT::resources_utils::find(name);
    auto urdf_string = OpenSoT::resources_utils::ReadFile(urdf_path->string());

    return XBot::ModelInterface::getModel(
        urdf_string,
        OPENSOT_TEST_MODEL_TYPE);
}

struct TestBase : ::testing::Test
{
    XBot::ModelInterface::Ptr _model_ptr;
    std::string _robot_name;

    TestBase(std::string robot_name):
        _robot_name(robot_name),
        _model_ptr(GetTestModel(robot_name))
    {
        std::cout << "model '" << _model_ptr->getName() <<
            "' nq = " << _model_ptr->getNq() << " nv = " << _model_ptr->getNv() <<
            std::endl;
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

#endif // COMMON_H
