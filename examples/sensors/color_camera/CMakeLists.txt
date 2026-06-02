add_executable(color-camera-example
    color-camera-example.cpp
)

target_compile_features(color-camera-example PRIVATE cxx_std_20)

hako_configure_target_warnings(color-camera-example)

target_include_directories(color-camera-example
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
)

target_include_directories(color-camera-example SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(color-camera-example
    PRIVATE
        msensors
)

target_compile_definitions(color-camera-example
    PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>
)

hako_configure_windows_runtime(color-camera-example)
