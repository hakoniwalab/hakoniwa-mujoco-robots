cmake_minimum_required(VERSION 3.20)

add_library(
    msensors
    imu/imu_sensor.cpp
    joint_state/joint_state_sensor.cpp
    lidar/lidar_2d_sensor.cpp
    noise/range_noise.cpp
    noise/axis_noise.cpp
    odometry/odometry_sensor.cpp
    tf/tf_publisher.cpp
)
target_compile_features(msensors PRIVATE cxx_std_20)
hako_configure_target_warnings(msensors)

target_include_directories(msensors
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)
target_include_directories(msensors SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(msensors
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
    ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)
target_compile_definitions(msensors PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
