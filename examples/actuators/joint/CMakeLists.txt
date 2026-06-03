add_executable(joint-actuator-example
    joint-actuator-example.cpp
)

target_compile_features(joint-actuator-example PRIVATE cxx_std_20)

hako_configure_target_warnings(joint-actuator-example)

target_include_directories(joint-actuator-example
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
)

target_include_directories(joint-actuator-example SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(joint-actuator-example
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
)

hako_configure_windows_runtime(joint-actuator-example)
