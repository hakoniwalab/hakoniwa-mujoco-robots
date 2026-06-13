# examples/sensors/ultrasonic/CMakeLists.txt

add_executable(ultrasonic-example
    ultrasonic-example.cpp
    support/ultrasonic_example_support.cpp
)
add_executable(ultrasonic-hakoniwa-asset
    ultrasonic-hakoniwa-asset.cpp
    support/ultrasonic_example_support.cpp
)

target_compile_features(ultrasonic-example PRIVATE cxx_std_20)
target_compile_features(ultrasonic-hakoniwa-asset PRIVATE cxx_std_20)

hako_configure_target_warnings(ultrasonic-example)
hako_configure_target_warnings(ultrasonic-hakoniwa-asset)

target_include_directories(ultrasonic-example
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)
target_include_directories(ultrasonic-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)

target_include_directories(ultrasonic-example SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)
target_include_directories(ultrasonic-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(ultrasonic-example
    PRIVATE
        msensors
)
target_link_libraries(ultrasonic-hakoniwa-asset
    PRIVATE
        msensors
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

target_compile_definitions(ultrasonic-example
    PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>
)
target_compile_definitions(ultrasonic-hakoniwa-asset
    PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>
)

hako_configure_windows_runtime(ultrasonic-example)
hako_configure_windows_runtime(ultrasonic-hakoniwa-asset)
