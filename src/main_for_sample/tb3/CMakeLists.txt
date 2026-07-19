cmake_minimum_required(VERSION 3.20)

set(TB3_SIM_SOURCES
    main.cpp
    ${PROJECT_ROOT_DIR}/src/config/asset_manifest.cpp
    ${PROJECT_ROOT_DIR}/src/runtime/hakoniwa_asset_lifecycle.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_drive.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_hakoniwa_adapter.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_robot.cpp
    ${PROJECT_ROOT_DIR}/src/robots/tb3/tb3_runtime_config_loader.cpp
)

function(add_tb3_sim_target target_name)
    add_executable(${target_name} ${TB3_SIM_SOURCES})
    target_compile_features(${target_name} PRIVATE cxx_std_20)
    hako_configure_target_warnings(${target_name})

    target_include_directories(${target_name}
        PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
        PRIVATE ${PROJECT_ROOT_DIR}/src
        PRIVATE ${PROJECT_ROOT_DIR}/include
        PRIVATE ${GLFW_DIR}
        PRIVATE ${HAKO_CORE_INCLUDE_BASE}
        PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
        PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    )
    target_include_directories(${target_name} SYSTEM PRIVATE
        ${MUJOCO_SOURCE_DIR}
        ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
        ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types
    )

    target_link_libraries(${target_name}
        msensors
        ${LIBMUJOCO}
        mujoco-common
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
    )
    target_compile_definitions(${target_name} PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
    hako_configure_windows_runtime(${target_name})
endfunction()

add_tb3_sim_target(tb3_sim)

add_tb3_sim_target(tb3_sim_burger)
target_compile_definitions(tb3_sim_burger PRIVATE
    HAKO_TB3_DEFAULT_MANIFEST_PATH="config/assets/tb3-mbody-burger-asset.json"
    HAKO_TB3_VIEWER_DISABLED_BY_DEFAULT=1
)
