cmake_minimum_required(VERSION 3.20)

option(HAKO_BUILD_SENSOR_TESTS "Build sensor tests" OFF)
option(HAKO_BUILD_CAMERA_SMOKE_TESTS "Build camera OpenGL/MuJoCo smoke tests" OFF)

add_library(
    msensors
    camera/camera_config_loader.cpp
    camera/camera_pdu_converter.cpp
    camera/camera_encoding_utils.cpp
    camera/image_frame_writer.cpp
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
    ultrasonic/ultrasonic_sensor.cpp
    debug/raycast_debug.cpp
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

function(hako_add_sensor_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_include_directories(${target_name}
        PRIVATE ${PROJECT_ROOT_DIR}
        PRIVATE ${PROJECT_ROOT_DIR}/src
        PRIVATE ${PROJECT_ROOT_DIR}/include
        PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    )
    target_include_directories(${target_name} SYSTEM PRIVATE
        ${MUJOCO_SOURCE_DIR}
        ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
    )
    target_link_libraries(${target_name} PRIVATE msensors)
endfunction()

if(HAKO_BUILD_SENSOR_TESTS)
    hako_add_sensor_test(
        camera_config_loader_test
        ${PROJECT_ROOT_DIR}/tests/sensors/camera/unit/camera_config_loader_test.cpp
    )
    hako_add_sensor_test(
        camera_pdu_converter_test
        ${PROJECT_ROOT_DIR}/tests/sensors/camera/unit/camera_pdu_converter_test.cpp
    )
    hako_add_sensor_test(
        depth_encoding_test
        ${PROJECT_ROOT_DIR}/tests/sensors/camera/unit/depth_encoding_test.cpp
    )
    hako_add_sensor_test(
        ultrasonic_config_loader_test
        ${PROJECT_ROOT_DIR}/tests/sensors/ultrasonic/unit/ultrasonic_config_loader_test.cpp
    )
    hako_add_sensor_test(
        ultrasonic_range_pdu_converter_test
        ${PROJECT_ROOT_DIR}/tests/sensors/ultrasonic/unit/ultrasonic_range_pdu_converter_test.cpp
    )
    hako_add_sensor_test(
        ultrasonic_measurement_test
        ${PROJECT_ROOT_DIR}/tests/sensors/ultrasonic/unit/ultrasonic_measurement_test.cpp
    )

    add_custom_target(
        camera_unit_tests
        DEPENDS
            camera_config_loader_test
            camera_pdu_converter_test
            depth_encoding_test
    )
    add_custom_target(
        ultrasonic_unit_tests
        DEPENDS
            ultrasonic_config_loader_test
            ultrasonic_range_pdu_converter_test
            ultrasonic_measurement_test
    )
    add_custom_target(
        sensor_unit_tests
        DEPENDS
            camera_unit_tests
            ultrasonic_unit_tests
    )
    add_custom_target(
        run_sensor_unit_tests
        COMMAND $<TARGET_FILE:camera_config_loader_test>
        COMMAND $<TARGET_FILE:camera_pdu_converter_test>
        COMMAND $<TARGET_FILE:depth_encoding_test>
        COMMAND $<TARGET_FILE:ultrasonic_config_loader_test>
        COMMAND $<TARGET_FILE:ultrasonic_range_pdu_converter_test>
        COMMAND $<TARGET_FILE:ultrasonic_measurement_test>
        DEPENDS sensor_unit_tests
        WORKING_DIRECTORY ${PROJECT_ROOT_DIR}
        USES_TERMINAL
    )
    add_custom_target(
        run_camera_unit_tests
        COMMAND $<TARGET_FILE:camera_config_loader_test>
        COMMAND $<TARGET_FILE:camera_pdu_converter_test>
        COMMAND $<TARGET_FILE:depth_encoding_test>
        DEPENDS camera_unit_tests
        WORKING_DIRECTORY ${PROJECT_ROOT_DIR}
        USES_TERMINAL
    )
endif()

if(HAKO_BUILD_CAMERA_SMOKE_TESTS)
    hako_add_sensor_test(
        depth_render_smoke_test
        ${PROJECT_ROOT_DIR}/tests/sensors/camera/smoke/depth_render_smoke_test.cpp
    )
    if(APPLE)
        target_link_libraries(depth_render_smoke_test PRIVATE "-framework ApplicationServices")
    endif()
    add_custom_target(
        camera_smoke_tests
        DEPENDS depth_render_smoke_test
    )
endif()
