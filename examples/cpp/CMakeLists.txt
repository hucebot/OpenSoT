add_executable(example_simple_qp simple_qp.cpp)
target_link_libraries(example_simple_qp OpenSoT OpenSotBackEndQPOases)

add_definitions(-DOPENSOT_TEST_MODEL_TYPE="pin")
    
add_executable(example_panda_ik panda_ik.cpp)
target_link_libraries(example_panda_ik OpenSoT ${qpOASES_LIBRARIES})

add_executable(example_go1_ik go1_ik.cpp)
target_link_libraries(example_go1_ik OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES})

