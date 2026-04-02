install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/robots
        DESTINATION share/OpenSoT/resources)

target_compile_definitions(OpenSoT
    PUBLIC OPENSOT_RESOURCES_PATH="${CMAKE_INSTALL_PREFIX}/share/OpenSoT/resources"
)

