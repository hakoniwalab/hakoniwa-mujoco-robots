add_executable(agilex-tracer-rover-example
    agilex-tracer-rover-example.cpp
)

target_compile_features(agilex-tracer-rover-example PRIVATE cxx_std_20)

hako_configure_target_warnings(agilex-tracer-rover-example)

target_include_directories(agilex-tracer-rover-example
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
)

target_include_directories(agilex-tracer-rover-example SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(agilex-tracer-rover-example
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
)

hako_configure_windows_runtime(agilex-tracer-rover-example)

add_executable(rover-twist-hakoniwa-asset
    rover-twist-hakoniwa-asset.cpp
)

target_compile_features(rover-twist-hakoniwa-asset PRIVATE cxx_std_20)

hako_configure_target_warnings(rover-twist-hakoniwa-asset)

target_include_directories(rover-twist-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-pdu-endpoint/include
)

target_include_directories(rover-twist-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(rover-twist-hakoniwa-asset
    PRIVATE
        ${LIBMUJOCO}
        mujoco-common
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

hako_configure_windows_runtime(rover-twist-hakoniwa-asset)
