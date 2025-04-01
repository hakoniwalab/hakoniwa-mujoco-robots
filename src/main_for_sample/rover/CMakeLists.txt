cmake_minimum_required(VERSION 3.20)

add_executable(
    rover_sim 
    main.cpp
    ${PROJECT_ROOT_DIR}/src/mujoco-common/mujoco_debug.cpp
    ${PROJECT_ROOT_DIR}/src/mujoco-common/mujoco_viewer.cpp
)

#MESSAGE(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

target_include_directories(rover_sim 
    PRIVATE ${MUJOCO_SOURCE_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${ROS2PDU_SOURCE_DIR}/pdu/types
    PRIVATE ${HAKO_CORE_SOURCE_DIR}
    )

# MuJoCoライブラリをリンク
target_link_libraries(rover_sim 
    ${LIBMUJOCO}
   glfw
    ${OPENGL_gl_LIBRARY}
    assets
    conductor
    "-framework OpenGL"
)