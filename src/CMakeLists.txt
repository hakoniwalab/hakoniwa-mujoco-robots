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
set(HAKONIWA_INSTALL_PREFIX "/usr/local/hakoniwa" CACHE PATH "Install prefix of hakoniwa-core-pro")

option(USE_MUJOCO "Enable MuJoCo support" ON)
if(USE_MUJOCO)
    add_definitions(-DMUJOCO_ENABLED)
    include(FetchContent)
    set(MUJOCO_VERSION_FILE "${PROJECT_ROOT_DIR}/MUJOCO_VERSION.txt")
    if(NOT EXISTS "${MUJOCO_VERSION_FILE}")
        message(FATAL_ERROR "MUJOCO version file not found: ${MUJOCO_VERSION_FILE}")
    endif()
    file(READ "${MUJOCO_VERSION_FILE}" MUJOCO_VERSION)
    string(STRIP "${MUJOCO_VERSION}" MUJOCO_VERSION)
    if(MUJOCO_VERSION STREQUAL "")
        message(FATAL_ERROR "MUJOCO_VERSION.txt is empty: ${MUJOCO_VERSION_FILE}")
    endif()
    set(MUJOCO_RELEASE_TAG "v${MUJOCO_VERSION}")
    message(STATUS "MUJOCO_VERSION=${MUJOCO_VERSION}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_SYSTEM_NAME STREQUAL "Windows")
        if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
            set(MUJOCO_PLATFORM "linux")
            set(MUJOCO_EXT "tar.gz")
            if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
                set(MUJOCO_ARCH "x86_64")
            elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
                set(MUJOCO_ARCH "aarch64")
            else()
                message(FATAL_ERROR "Unsupported Linux architecture for MuJoCo: ${CMAKE_SYSTEM_PROCESSOR}")
            endif()
        else()
            set(MUJOCO_PLATFORM "windows")
            set(MUJOCO_EXT "zip")
            if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
                set(MUJOCO_ARCH "x86_64")
            else()
                message(FATAL_ERROR "Unsupported Windows architecture for MuJoCo: ${CMAKE_SYSTEM_PROCESSOR}")
            endif()
        endif()

        set(MUJOCO_PACKAGE_PATH "mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}-${MUJOCO_ARCH}.${MUJOCO_EXT}")
        set(MUJOCO_PACKAGE_URL_VTAG "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_RELEASE_TAG}/${MUJOCO_PACKAGE_PATH}")
        set(MUJOCO_PACKAGE_URL_PLAIN "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_PACKAGE_PATH}")
        message(STATUS "Attempting to download MuJoCo package: ${MUJOCO_PACKAGE_URL_VTAG}")

        FetchContent_Declare(
            mujoco_precompiled
            URL "${MUJOCO_PACKAGE_URL_VTAG}" "${MUJOCO_PACKAGE_URL_PLAIN}"
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
        FetchContent_MakeAvailable(mujoco_precompiled)
        FetchContent_GetProperties(mujoco_precompiled)
        if(NOT mujoco_precompiled_POPULATED)
            FetchContent_Populate(mujoco_precompiled)
        endif()
        set(MUJOCO_SOURCE_DIR "${mujoco_precompiled_SOURCE_DIR}/include")
        set(MUJOCO_LIB_DIR "${mujoco_precompiled_SOURCE_DIR}/lib")
        set(LIBMUJOCO "mujoco")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        # macOS releases are published as DMG, so we download and copy include/lib.
        set(MUJOCO_PACKAGE_PATH "mujoco-${MUJOCO_VERSION}-macos-universal2.dmg")
        set(MUJOCO_PACKAGE_URL_VTAG "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_RELEASE_TAG}/${MUJOCO_PACKAGE_PATH}")
        set(MUJOCO_PACKAGE_URL_PLAIN "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_PACKAGE_PATH}")
        message(STATUS "Attempting to download MuJoCo package: ${MUJOCO_PACKAGE_URL_VTAG}")

        set(MUJOCO_DMG_PATH "${CMAKE_BINARY_DIR}/_deps/mujoco-${MUJOCO_VERSION}-macos-universal2.dmg")
        set(MUJOCO_MOUNT_DIR "${CMAKE_BINARY_DIR}/_deps/mujoco_mount")
        set(MUJOCO_SOURCE_ROOT_DIR "${CMAKE_BINARY_DIR}/_deps/mujoco_precompiled-src")
        set(MUJOCO_SOURCE_DIR "${MUJOCO_SOURCE_ROOT_DIR}/include")
        set(MUJOCO_LIB_DIR "${MUJOCO_SOURCE_ROOT_DIR}/lib")
        set(MUJOCO_FRAMEWORK_DIR "${MUJOCO_LIB_DIR}/mujoco.framework")
        set(MUJOCO_DYLIB_PATH "${MUJOCO_FRAMEWORK_DIR}/Versions/A/libmujoco.${MUJOCO_VERSION}.dylib")

        if(NOT EXISTS "${MUJOCO_SOURCE_DIR}/mujoco/mujoco.h" OR NOT EXISTS "${MUJOCO_DYLIB_PATH}")
            set(MUJOCO_DOWNLOAD_CODE 1)
            foreach(_url IN ITEMS "${MUJOCO_PACKAGE_URL_VTAG}" "${MUJOCO_PACKAGE_URL_PLAIN}")
                file(DOWNLOAD "${_url}" "${MUJOCO_DMG_PATH}" SHOW_PROGRESS STATUS MUJOCO_DOWNLOAD_STATUS LOG MUJOCO_DOWNLOAD_LOG)
                list(GET MUJOCO_DOWNLOAD_STATUS 0 MUJOCO_DOWNLOAD_CODE)
                list(GET MUJOCO_DOWNLOAD_STATUS 1 MUJOCO_DOWNLOAD_MSG)
                if(MUJOCO_DOWNLOAD_CODE EQUAL 0)
                    break()
                endif()
            endforeach()
            if(NOT MUJOCO_DOWNLOAD_CODE EQUAL 0)
                message(FATAL_ERROR
                    "Failed to download MuJoCo package.\n"
                    "URLs tried:\n  - ${MUJOCO_PACKAGE_URL_VTAG}\n  - ${MUJOCO_PACKAGE_URL_PLAIN}\n"
                    "status: ${MUJOCO_DOWNLOAD_CODE} (${MUJOCO_DOWNLOAD_MSG})\n"
                    "If URL returns 404, the version in MUJOCO_VERSION.txt may not exist."
                )
            endif()

            file(MAKE_DIRECTORY "${MUJOCO_MOUNT_DIR}")
            execute_process(COMMAND hdiutil detach "${MUJOCO_MOUNT_DIR}" RESULT_VARIABLE _detach_before OUTPUT_QUIET ERROR_QUIET)
            execute_process(
                COMMAND hdiutil attach "${MUJOCO_DMG_PATH}" -readonly -nobrowse -mountpoint "${MUJOCO_MOUNT_DIR}"
                RESULT_VARIABLE MUJOCO_ATTACH_RESULT
                ERROR_VARIABLE MUJOCO_ATTACH_ERROR
            )
            if(NOT MUJOCO_ATTACH_RESULT EQUAL 0)
                message(FATAL_ERROR "Failed to mount MuJoCo dmg: ${MUJOCO_ATTACH_ERROR}")
            endif()

            file(MAKE_DIRECTORY "${MUJOCO_SOURCE_ROOT_DIR}")
            if(EXISTS "${MUJOCO_MOUNT_DIR}/include" AND EXISTS "${MUJOCO_MOUNT_DIR}/lib")
                execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory "${MUJOCO_MOUNT_DIR}/include" "${MUJOCO_SOURCE_DIR}")
                execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory "${MUJOCO_MOUNT_DIR}/lib" "${MUJOCO_LIB_DIR}")
            elseif(EXISTS "${MUJOCO_MOUNT_DIR}/mujoco.framework/Versions/A/Headers")
                # MuJoCo 3.5.0 macOS DMG layout: headers and dylib are in mujoco.framework.
                file(MAKE_DIRECTORY "${MUJOCO_SOURCE_DIR}/mujoco")
                file(MAKE_DIRECTORY "${MUJOCO_LIB_DIR}")
                execute_process(
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${MUJOCO_MOUNT_DIR}/mujoco.framework/Versions/A/Headers"
                    "${MUJOCO_SOURCE_DIR}/mujoco"
                )
                execute_process(
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${MUJOCO_MOUNT_DIR}/mujoco.framework"
                    "${MUJOCO_FRAMEWORK_DIR}"
                )
            else()
                execute_process(COMMAND hdiutil detach "${MUJOCO_MOUNT_DIR}" OUTPUT_QUIET ERROR_QUIET)
                message(FATAL_ERROR
                    "Unexpected MuJoCo dmg layout in ${MUJOCO_MOUNT_DIR}. "
                    "Expected either include/lib or mujoco.framework/Versions/A."
                )
            endif()
            execute_process(COMMAND hdiutil detach "${MUJOCO_MOUNT_DIR}" OUTPUT_QUIET ERROR_QUIET)
        endif()

        if(EXISTS "${MUJOCO_DYLIB_PATH}")
            set(LIBMUJOCO "${MUJOCO_DYLIB_PATH}")
        else()
            set(LIBMUJOCO "mujoco")
        endif()
    else()
        message(FATAL_ERROR "Unsupported OS for MuJoCo auto-download: ${CMAKE_SYSTEM_NAME}")
    endif()

    include_directories("${MUJOCO_SOURCE_DIR}")
    link_directories("${MUJOCO_LIB_DIR}")
    message(STATUS "MuJoCo v${MUJOCO_VERSION} downloaded and configured.")
else()
    message(STATUS "MuJoCo disabled: Using simple physics model")
endif()

set(HAKO_CORE_INCLUDE_ROOT "${HAKONIWA_INSTALL_PREFIX}/include/hakoniwa")
set(HAKO_CORE_INCLUDE_BASE "${HAKONIWA_INSTALL_PREFIX}/include")
set(HAKO_PDU_TYPES_INCLUDE_DIR "${HAKONIWA_INSTALL_PREFIX}/include/hakoniwa/pdu")
set(HAKO_CORE_LIBRARY_DIR "${HAKONIWA_INSTALL_PREFIX}/lib")

list(APPEND CMAKE_PREFIX_PATH "${HAKONIWA_INSTALL_PREFIX}")
find_package(hakoniwa-core CONFIG QUIET)
if(hakoniwa-core_FOUND)
    message(STATUS "Using installed hakoniwa-core package: ${HAKONIWA_INSTALL_PREFIX}")
    set(HAKO_ASSETS_LINK_TARGET hakoniwa-core::assets)
    set(HAKO_CONDUCTOR_LINK_TARGET hakoniwa-core::conductor)
else()
    message(STATUS "hakoniwa-core package config not found. Falling back to direct library search in ${HAKO_CORE_LIBRARY_DIR}")
    find_library(HAKO_ASSETS_LIB NAMES assets PATHS "${HAKO_CORE_LIBRARY_DIR}" REQUIRED NO_DEFAULT_PATH)
    find_library(HAKO_CONDUCTOR_LIB NAMES conductor PATHS "${HAKO_CORE_LIBRARY_DIR}" REQUIRED NO_DEFAULT_PATH)
    set(HAKO_ASSETS_LINK_TARGET "${HAKO_ASSETS_LIB}")
    set(HAKO_CONDUCTOR_LINK_TARGET "${HAKO_CONDUCTOR_LIB}")
endif()

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


add_subdirectory(main_for_sample/forklift)
add_subdirectory(main_for_sample/rover)
