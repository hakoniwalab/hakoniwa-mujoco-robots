cmake_minimum_required(VERSION 3.20)

add_executable(
    forklift_sim 
    main.cpp
)

#MESSAGE(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

target_include_directories(forklift_sim 
    PRIVATE ${MUJOCO_SOURCE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${ROS2PDU_SOURCE_DIR}/pdu/types
    PRIVATE ${HAKO_CORE_SOURCE_DIR}
    )

# MuJoCoライブラリをリンク
target_link_libraries(forklift_sim 
    ${LIBMUJOCO}
    mujoco-common
    assets
    conductor
)
target_compile_definitions(forklift_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
