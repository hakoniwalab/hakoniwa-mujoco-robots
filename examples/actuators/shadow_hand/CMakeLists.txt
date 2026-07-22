add_executable(shadow-hand-actuator-map
    shadow-hand-actuator-map.cpp
)

target_compile_features(shadow-hand-actuator-map PRIVATE cxx_std_20)

hako_configure_target_warnings(shadow-hand-actuator-map)

target_include_directories(shadow-hand-actuator-map
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
)

target_include_directories(shadow-hand-actuator-map SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
)

target_link_libraries(shadow-hand-actuator-map
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
)

hako_configure_windows_runtime(shadow-hand-actuator-map)

add_executable(shadow-hand-hakoniwa-asset
    shadow-hand-hakoniwa-asset.cpp
)

target_compile_features(shadow-hand-hakoniwa-asset PRIVATE cxx_std_20)

hako_configure_target_warnings(shadow-hand-hakoniwa-asset)

target_include_directories(shadow-hand-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-pdu-endpoint/include
)

target_include_directories(shadow-hand-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
)

target_link_libraries(shadow-hand-hakoniwa-asset
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
        msensors
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

hako_configure_windows_runtime(shadow-hand-hakoniwa-asset)
