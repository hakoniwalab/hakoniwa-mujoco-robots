cmake_minimum_required(VERSION 3.20)

add_executable(
    forklift_sim 
    main.cpp
)

add_executable(
    forklift_unit_sim
    main_unit.cpp
    forklift_unit_app.cpp
)

#MESSAGE(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

target_include_directories(forklift_sim 
    PRIVATE ${MUJOCO_SOURCE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    )

# MuJoCoライブラリをリンク
target_link_libraries(forklift_sim 
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
)
target_compile_definitions(forklift_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)

target_include_directories(forklift_unit_sim
    PRIVATE ${MUJOCO_SOURCE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
    )

target_link_libraries(forklift_unit_sim
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
)
target_compile_definitions(forklift_unit_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
