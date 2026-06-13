add_executable(color-camera-example
    color-camera-example.cpp
    support/color_camera_example_support.cpp
)
add_executable(color-camera-hakoniwa-asset
    color-camera-hakoniwa-asset.cpp
    support/color_camera_example_support.cpp
)

target_compile_features(color-camera-example PRIVATE cxx_std_20)
target_compile_features(color-camera-hakoniwa-asset PRIVATE cxx_std_20)

hako_configure_target_warnings(color-camera-example)
hako_configure_target_warnings(color-camera-hakoniwa-asset)

target_include_directories(color-camera-example
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
)
target_include_directories(color-camera-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)

target_include_directories(color-camera-example SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)
target_include_directories(color-camera-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(color-camera-example
    PRIVATE
        msensors
)
target_link_libraries(color-camera-hakoniwa-asset
    PRIVATE
        msensors
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

target_compile_definitions(color-camera-example
    PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>
)
target_compile_definitions(color-camera-hakoniwa-asset
    PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>
)

hako_configure_windows_runtime(color-camera-example)
hako_configure_windows_runtime(color-camera-hakoniwa-asset)
