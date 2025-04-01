cmake_minimum_required(VERSION 3.10...3.25)

project(hakoniwa-mujoco-robots-root
    VERSION 1.0.0
    DESCRIPTION "hakoniwa-mujoco-robots project"
    LANGUAGES C CXX
)
if (DEFINED HAKO_CMAKE_OPTION_FILEPATH)
    message(STATUS "HAKO_CMAKE_OPTION_FILEPATH=${HAKO_CMAKE_OPTION_FILEPATH}")
    include(${HAKO_CMAKE_OPTION_FILEPATH})
else()
    message(STATUS "HAKO_CMAKE_OPTION_FILEPATH is not defined. Use default options.")
    set(HAKO_CMAKE_OPTION_FILEPATH "${PROJECT_SOURCE_DIR}/cmake-options/default-cmake-options.cmake")
    include(cmake-options/default-cmake-options.cmake)
endif()

set(PROJECT_ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
set(ROS2PDU_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/hakoniwa-ros2pdu")
set(MUJOCO_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/mujoco/include")
set(HAKO_CORE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/hakoniwa-core-cpp-client/src")
set(HAKONIWA_PUBLIC_LIB_DIR "/usr/local/lib/hakoniwa")

option(USE_MUJOCO "Enable MuJoCo support" ON)
if(USE_MUJOCO)
    add_definitions(-DMUJOCO_ENABLED)
    set(LIBMUJOCO "mujoco.3.3.0")

    if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
        #TODO
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        link_directories(${HAKONIWA_PUBLIC_LIB_DIR})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        #TODO
    endif()
    message(STATUS "MuJoCo enabled: Linking with libmujoco")
else()
    message(STATUS "MuJoCo disabled: Using simple physics model")
endif()

set(HAKO_CORE_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/hakoniwa-core-cpp-client")

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_subdirectory(
    ${HAKO_CORE_DIR}
    ${CMAKE_BINARY_DIR}/thirdparty/hakoniwa-core-cpp-client
    )

add_subdirectory(main_for_sample/forklift)
add_subdirectory(main_for_sample/rover)
