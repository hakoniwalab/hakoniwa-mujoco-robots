cmake_minimum_required(VERSION 3.20)

add_executable(
    forklift_sim 
    main.cpp
)
target_compile_features(forklift_sim PRIVATE cxx_std_20)
hako_configure_target_warnings(forklift_sim)

add_executable(
    forklift_unit_sim
    main_unit.cpp
    forklift_unit_app.cpp
    rd_light_integration.cpp
    forklift_visibility_controller.cpp
    forklift_trace_logger.cpp
    forklift_recovery_logger.cpp
    forklift_pdu_runtime.cpp
    forklift_simulation_loop.cpp
)
target_compile_features(forklift_unit_sim PRIVATE cxx_std_20)
hako_configure_target_warnings(forklift_unit_sim)

#MESSAGE(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

target_include_directories(forklift_sim 
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    )
target_include_directories(forklift_sim SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

# MuJoCoライブラリをリンク
target_link_libraries(forklift_sim 
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
)
target_compile_definitions(forklift_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
hako_configure_windows_runtime(forklift_sim)

target_include_directories(forklift_unit_sim
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    )
target_include_directories(forklift_unit_sim SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(forklift_unit_sim
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
)
target_compile_definitions(forklift_unit_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
hako_configure_windows_runtime(forklift_unit_sim)
