cmake_minimum_required(VERSION 3.20)

add_executable(
    rover_sim 
    main.cpp
)

#MESSAGE(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

target_include_directories(rover_sim 
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
target_link_libraries(rover_sim 
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
)
target_compile_definitions(rover_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
