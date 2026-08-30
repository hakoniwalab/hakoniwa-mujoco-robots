add_executable(generic-ackermann-hakoniwa-asset
    generic-ackermann-hakoniwa-asset.cpp
    ${PROJECT_ROOT_DIR}/src/config/asset_manifest.cpp
    ${PROJECT_ROOT_DIR}/src/robots/ackermann/ackermann_runtime_config.cpp
    ${PROJECT_ROOT_DIR}/src/runtime/hakoniwa_asset_lifecycle.cpp
)

target_compile_features(generic-ackermann-hakoniwa-asset PRIVATE cxx_std_20)
hako_configure_target_warnings(generic-ackermann-hakoniwa-asset)

target_include_directories(generic-ackermann-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-pdu-endpoint/include
)

target_include_directories(generic-ackermann-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(generic-ackermann-hakoniwa-asset
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

hako_configure_windows_runtime(generic-ackermann-hakoniwa-asset)
