cmake_minimum_required(VERSION 3.20)

option(HAKO_BUILD_SENSOR_TESTS "Build sensor tests" OFF)

add_library(
    msensors
    camera/camera_config_loader.cpp
    camera/camera_encoding_utils.cpp
    camera/camera_sensor.cpp
    camera/depth_camera_sensor.cpp
    camera/mujoco_camera_renderer.cpp
    camera/rgbd_camera_sensor.cpp
    camera/stereo_camera_sensor.cpp
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

if(HAKO_BUILD_SENSOR_TESTS)
    add_executable(
        depth_encoding_unit_test
        ${PROJECT_ROOT_DIR}/tests/sensors/camera/depth_encoding_unit_test.cpp
    )
    target_include_directories(depth_encoding_unit_test
        PRIVATE ${PROJECT_ROOT_DIR}/src
        PRIVATE ${PROJECT_ROOT_DIR}/include
    )
    target_include_directories(depth_encoding_unit_test SYSTEM PRIVATE
        ${MUJOCO_SOURCE_DIR}
    )
    target_link_libraries(depth_encoding_unit_test PRIVATE msensors)

    add_executable(
        depth_render_smoke_test
        ${PROJECT_ROOT_DIR}/tests/sensors/camera/depth_render_smoke_test.cpp
    )
    target_include_directories(depth_render_smoke_test
        PRIVATE ${PROJECT_ROOT_DIR}/src
        PRIVATE ${PROJECT_ROOT_DIR}/include
    )
    target_include_directories(depth_render_smoke_test SYSTEM PRIVATE
        ${MUJOCO_SOURCE_DIR}
    )
    target_link_libraries(depth_render_smoke_test PRIVATE msensors)
    if(APPLE)
        target_link_libraries(depth_render_smoke_test PRIVATE "-framework ApplicationServices")
    endif()

    add_executable(
        camera_config_loader_unit_test
        ${PROJECT_ROOT_DIR}/tests/sensors/camera/camera_config_loader_unit_test.cpp
    )
    target_include_directories(camera_config_loader_unit_test
        PRIVATE ${PROJECT_ROOT_DIR}/src
        PRIVATE ${PROJECT_ROOT_DIR}/include
    )
    target_include_directories(camera_config_loader_unit_test SYSTEM PRIVATE
        ${MUJOCO_SOURCE_DIR}
        ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
    )
    target_link_libraries(camera_config_loader_unit_test PRIVATE msensors)
endif()
