cmake_minimum_required(VERSION 3.16)

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
set(HAKO_CORE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/hakoniwa-core-cpp-client/src")

option(USE_MUJOCO "Enable MuJoCo support" ON)
if(USE_MUJOCO)
    add_definitions(-DMUJOCO_ENABLED)
    set(MUJOCO_VERSION "3.3.6")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_SYSTEM_NAME STREQUAL "Windows")
        # For Linux and Windows, download automatically
        message(STATUS "Linux/Windows detected. Attempting to download pre-compiled MuJoCo...")
        include(FetchContent)

        if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
            set(MUJOCO_PLATFORM "linux")
            set(MUJOCO_EXT "tar.gz")
        else()
            set(MUJOCO_PLATFORM "windows")
            set(MUJOCO_EXT "zip")
        endif()

        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
            set(MUJOCO_ARCH "x86_64")
        elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
            set(MUJOCO_ARCH "aarch64")
        endif()

        FetchContent_Declare(
            mujoco_precompiled
            URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}-${MUJOCO_ARCH}.${MUJOCO_EXT}"
        )
        FetchContent_MakeAvailable(mujoco_precompiled)
        FetchContent_GetProperties(mujoco_precompiled)
        if(NOT mujoco_precompiled_POPULATED)
            FetchContent_Populate(mujoco_precompiled)
        endif()

        set(MUJOCO_SOURCE_DIR ${mujoco_precompiled_SOURCE_DIR}/include)
        set(LIBMUJOCO "mujoco")
        include_directories(${MUJOCO_SOURCE_DIR})
        link_directories(${mujoco_precompiled_SOURCE_DIR}/lib)
        message(STATUS "MuJoCo v${MUJOCO_VERSION} downloaded and configured.")

    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        # For macOS, require manual installation into a project-local directory
        message(STATUS "macOS detected. Looking for local MuJoCo library.")
        set(MUJOCO_INSTALL_DIR "${PROJECT_SOURCE_DIR}/../vendor/mujoco" CACHE PATH "Path to pre-installed MuJoCo directory for macOS")

        set(MUJOCO_HEADER_FILE "${MUJOCO_INSTALL_DIR}/include/mujoco/mujoco.h")
        set(MUJOCO_LIBRARY_FILE "${MUJOCO_INSTALL_DIR}/lib/libmujoco.${MUJOCO_VERSION}.dylib")

        if(NOT EXISTS ${MUJOCO_HEADER_FILE} OR NOT EXISTS ${MUJOCO_LIBRARY_FILE})
            message(FATAL_ERROR "\nMuJoCo not found in '${MUJOCO_INSTALL_DIR}'.\nOn macOS, please download the 'mujoco-*-macos-universal2.dmg' from https://github.com/google-deepmind/mujoco/releases.\nThen, create a 'vendor/mujoco' directory in your project root and copy the 'include' and 'lib' folders from the DMG into it.\nAlternatively, specify a custom path by setting -DMUJOCO_INSTALL_DIR=/path/to/your/mujoco_dir during cmake configuration.")
        endif()

        message(STATUS "Found MuJoCo at: ${MUJOCO_INSTALL_DIR}")
        set(MUJOCO_SOURCE_DIR ${MUJOCO_INSTALL_DIR}/include)
        set(LIBMUJOCO ${MUJOCO_LIBRARY_FILE})
        include_directories(${MUJOCO_SOURCE_DIR})
        link_directories(${MUJOCO_INSTALL_DIR}/lib)
    endif()
else()
    message(STATUS "MuJoCo disabled: Using simple physics model")
endif()

set(HAKO_CORE_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/hakoniwa-core-cpp-client")

option(USE_VIEWER "Enable MuJoCo viewer" ON)
MESSAGE(STATUS "USE_VIEWER=${USE_VIEWER}")
if(USE_VIEWER)
    MESSAGE(STATUS "MuJoCo viewer enabled")
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    add_library(mujoco-common STATIC mujoco-common/mujoco_viewer.cpp mujoco-common/mujoco_debug.cpp)
    target_include_directories(mujoco-common PUBLIC mujoco-common)
    if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        target_link_libraries(mujoco-common PUBLIC ${OpenGL_LIBRARIES} "-framework OpenGL" glfw)
    else()
        target_link_libraries(mujoco-common PUBLIC OpenGL::GL glfw)
    endif()
else()
    MESSAGE(STATUS "MuJoCo viewer disabled")
    add_library(mujoco-common STATIC mujoco-common/mujoco_debug.cpp)
    target_include_directories(mujoco-common PUBLIC mujoco-common)
endif()


add_subdirectory(
    ${HAKO_CORE_DIR}
    ${CMAKE_BINARY_DIR}/thirdparty/hakoniwa-core-cpp-client
    )

add_subdirectory(main_for_sample/forklift)
add_subdirectory(main_for_sample/rover)
