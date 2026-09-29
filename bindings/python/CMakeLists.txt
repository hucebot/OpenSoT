cmake_minimum_required(VERSION 3.20)

find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
find_package(pybind11 QUIET)
find_package(Eigen3 REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

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
                                    variables/torque.hpp)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Eigen3_INCLUDE_DIR})
    pybind11_add_module(_pyopensot ${OPENSOT_BINDINGS_SOURCES})

    target_link_libraries(_pyopensot PUBLIC OpenSoT)

    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    if(SKBUILD)
        set(OPENSOT_PY_PKG_DIR "${SKBUILD_PLATLIB_DIR}/pyopensot")
        set(OPENSOT_PY_PKG_ROOT_DIR "${SKBUILD_PLATLIB_DIR}")
    else()
        set(OPENSOT_PY_PKG_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/pyopensot")
        set(OPENSOT_PY_PKG_ROOT_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
    endif()

    install(TARGETS _pyopensot
        LIBRARY DESTINATION "${OPENSOT_PY_PKG_DIR}"
        ARCHIVE DESTINATION "${OPENSOT_PY_PKG_DIR}"
        RUNTIME DESTINATION "${OPENSOT_PY_PKG_DIR}")

    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/pyopensot/__init__.py
            DESTINATION "${OPENSOT_PY_PKG_DIR}")

    if(${OPENSOT_COMPILE_COLLISION})
        set(OPENSOT_BINDINGS_COLLISION_SOURCES pyopensot_collision.cpp constraints/velocity_collision.hpp)

        include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        pybind11_add_module(pyopensot_collision ${OPENSOT_BINDINGS_COLLISION_SOURCES})

        target_link_libraries(pyopensot_collision PUBLIC OpenSoT)

        find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

        install(TARGETS pyopensot_collision
            COMPONENT python
            LIBRARY DESTINATION "${OPENSOT_PY_PKG_ROOT_DIR}"
            ARCHIVE DESTINATION "${OPENSOT_PY_PKG_ROOT_DIR}"
            RUNTIME DESTINATION "${OPENSOT_PY_PKG_ROOT_DIR}")
    endif()

    install(
        DIRECTORY viser_opensot_tools
        DESTINATION "${OPENSOT_PY_PKG_DIR}"
    )
    # TODO is it useful / used ?
    install(
        PROGRAMS viser_opensot_tools/replay
        DESTINATION "${OPENSOT_PY_PKG_DIR}"
    )

else()
    message(STATUS "pybind11 not found")
endif()

# XXX TODO installer tout proprement... les resources, etc. Ne pas installer les include, .cmake, etc par défaut dans pip (?)
# XXX je mets ça ici pour que ça crashe si je relance ...
# XXX where do we put viser stuff ?
# XXX rendre viser et ressources optionnel dans pip: pip install pyopensot[viz]  ou [resources...]

