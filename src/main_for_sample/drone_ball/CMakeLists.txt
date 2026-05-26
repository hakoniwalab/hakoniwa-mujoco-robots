cmake_minimum_required(VERSION 3.20)

add_executable(
    drone_ball_sim
    main.cpp
)
target_compile_features(drone_ball_sim PRIVATE cxx_std_20)
hako_configure_target_warnings(drone_ball_sim)

target_include_directories(drone_ball_sim
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)
target_include_directories(drone_ball_sim SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
    ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types
)

target_link_libraries(drone_ball_sim
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
    ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)
target_compile_definitions(drone_ball_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
hako_configure_windows_runtime(drone_ball_sim)
