cmake_minimum_required(VERSION 3.20)

find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
find_package(pybind11 QUIET)
find_package(Eigen3 REQUIRED)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    set(OPENSOT_BINDINGS_SOURCES    pyopensot.cpp
                                    variables.hpp
                                    autostack.hpp
                                    sub.hpp
                                    aggregated.hpp
                                    generic.hpp
                                    basic.hpp
                                    solver.hpp
                                    tasks/velocity.hpp
                                    tasks/acceleration.hpp
                                    tasks/minimize_variable.hpp
                                    constraints/velocity.hpp
                                    constraints/acceleration.hpp
                                    constraints/force.hpp
                                    variables/torque.hpp
                                    oc.hpp
                                    liegroupsutils.hpp)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Eigen3_INCLUDE_DIR})

    if(${OPENSOT_COMPILE_COLLISION})
        set(OPENSOT_BINDINGS_SOURCES ${OPENSOT_BINDINGS_SOURCES} constraints/velocity_collision.hpp)
    endif()
    if(${OPENSOT_SOTH_FRONT_END})
        set(OPENSOT_BINDINGS_SOURCES ${OPENSOT_BINDINGS_SOURCES} solver_hcod.hpp)
    endif()
    if(${hpipm-cpp_FOUND})
        set(OPENSOT_BINDINGS_SOURCES ${OPENSOT_BINDINGS_SOURCES} solver_hpipmoc.hpp)
    endif()

    pybind11_add_module(pyopensot ${OPENSOT_BINDINGS_SOURCES})

    target_compile_definitions(pyopensot PRIVATE $<$<BOOL:${OPENSOT_COMPILE_COLLISION}>:OPENSOT_COMPILE_COLLISION>)
    target_compile_definitions(pyopensot PRIVATE $<$<BOOL:${OPENSOT_SOTH_FRONT_END}>:OPENSOT_SOTH_FRONT_END>)
    target_compile_definitions(pyopensot PRIVATE $<$<BOOL:${hpipm-cpp_FOUND}>:HPIPM_CPP_FOUND>)

    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    target_link_libraries(pyopensot PUBLIC OpenSoT)
    if(${hpipm-cpp_FOUND})
        target_link_libraries(pyopensot PUBLIC OpenSothpipmOC)
    endif()

    # Check if pybind11-stubgen is available
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import pybind11_stubgen"
        RESULT_VARIABLE STUBGEN_AVAILABLE
        OUTPUT_QUIET ERROR_QUIET
    )

    set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

    install(TARGETS pyopensot
        COMPONENT python
        LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "bin")

    # Generate stubs during install
    if(STUBGEN_AVAILABLE EQUAL 0)
        install(CODE "
            message(STATUS \"Generating Python stub files for pyopensot\")
            execute_process(
                COMMAND ${Python3_EXECUTABLE} -m pybind11_stubgen 
                    pyopensot
                    -o ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
                WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
                RESULT_VARIABLE STUB_RESULT
            )
            if(NOT STUB_RESULT EQUAL 0)
                message(WARNING \"Failed to generate stub files\")
            endif()
        "
        COMPONENT python)
        message(STATUS "Stub generation will run during install")
    else()
        message(STATUS "pybind11-stubgen not found. Install with: pip install pybind11-stubgen")
    endif()

else()
    message(STATUS "pybind not found")
endif()