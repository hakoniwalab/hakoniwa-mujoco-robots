cmake_minimum_required(VERSION 3.20)

add_executable(
    tb3_sim
    main.cpp
)
target_compile_features(tb3_sim PRIVATE cxx_std_20)

target_include_directories(tb3_sim
    PRIVATE ${MUJOCO_SOURCE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)

target_link_libraries(tb3_sim
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
    ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)
target_compile_definitions(tb3_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
