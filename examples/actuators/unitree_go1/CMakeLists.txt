add_executable(unitree-go1-joint-io-example
    unitree-go1-joint-io-example.cpp
)

target_compile_features(unitree-go1-joint-io-example PRIVATE cxx_std_20)

hako_configure_target_warnings(unitree-go1-joint-io-example)

target_include_directories(unitree-go1-joint-io-example
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
)

target_include_directories(unitree-go1-joint-io-example SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
)

target_link_libraries(unitree-go1-joint-io-example
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
)

hako_configure_windows_runtime(unitree-go1-joint-io-example)

add_executable(unitree-go1-joint-hakoniwa-asset
    unitree-go1-joint-hakoniwa-asset.cpp
)

target_compile_features(unitree-go1-joint-hakoniwa-asset PRIVATE cxx_std_20)

hako_configure_target_warnings(unitree-go1-joint-hakoniwa-asset)

target_include_directories(unitree-go1-joint-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-pdu-endpoint/include
)

target_include_directories(unitree-go1-joint-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
)

target_link_libraries(unitree-go1-joint-hakoniwa-asset
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
        msensors
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

hako_configure_windows_runtime(unitree-go1-joint-hakoniwa-asset)
