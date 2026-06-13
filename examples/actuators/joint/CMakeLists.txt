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

add_executable(joint-actuator-hakoniwa-asset
    joint-actuator-hakoniwa-asset.cpp
)

target_compile_features(joint-actuator-hakoniwa-asset PRIVATE cxx_std_20)

hako_configure_target_warnings(joint-actuator-hakoniwa-asset)

target_include_directories(joint-actuator-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-pdu-endpoint/include
)

target_include_directories(joint-actuator-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(joint-actuator-hakoniwa-asset
    PRIVATE
        mujoco-common
        ${LIBMUJOCO}
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

hako_configure_windows_runtime(joint-actuator-hakoniwa-asset)
