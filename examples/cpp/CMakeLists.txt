add_executable(example_simple_qp simple_qp.cpp)
target_link_libraries(example_simple_qp OpenSoT OpenSotBackEndQPOases)

add_definitions(-DOPENSOT_TEST_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
set(OPENSOT_TEST_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
message(STATUS "OPENSOT_TEST_PATH is ${OPENSOT_TEST_PATH}")
add_definitions(-DOPENSOT_TEST_MODEL_TYPE="pin")

find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
    
add_executable(example_panda_ik panda_ik.cpp)
target_link_libraries(example_panda_ik OpenSoT ${qpOASES_LIBRARIES})

add_executable(example_coman_ik coman_ik.cpp)
target_link_libraries(example_coman_ik OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} rclcpp::rclcpp tf2_eigen::tf2_eigen tf2_ros::tf2_ros sensor_msgs::sensor_msgs_library)

