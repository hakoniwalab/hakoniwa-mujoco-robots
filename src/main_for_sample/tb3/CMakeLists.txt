cmake_minimum_required(VERSION 3.20)

add_executable(
    tb3_sim
    main.cpp
    ${PROJECT_ROOT_DIR}/src/config/asset_manifest.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_drive.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_hakoniwa_adapter.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_robot.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_runtime_config_loader.cpp
)
target_compile_features(tb3_sim PRIVATE cxx_std_20)
hako_configure_target_warnings(tb3_sim)

target_include_directories(tb3_sim
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)
target_include_directories(tb3_sim SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
    ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types
)

target_link_libraries(tb3_sim
    msensors
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
    ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)
target_compile_definitions(tb3_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
hako_configure_windows_runtime(tb3_sim)
