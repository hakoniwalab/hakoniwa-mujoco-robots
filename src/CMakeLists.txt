cmake_minimum_required(VERSION 3.16)

project(hakoniwa-mujoco-robots-root
    VERSION 1.0.0
    DESCRIPTION "hakoniwa-mujoco-robots project"
    LANGUAGES C CXX
)

function(hako_configure_target_warnings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE
            /W4
            /external:W0
            /wd4710
            /wd4711
            /wd5220
            /wd5247
        )
    endif()
endfunction()

function(hako_define_windows_mujoco_target)
    if(NOT WIN32 OR NOT USE_MUJOCO)
        return()
    endif()
    if(TARGET MuJoCo::mujoco)
        return()
    endif()

    if(MUJOCO_ROOT)
        set(_hako_mujoco_base_dir "${MUJOCO_ROOT}")
    else()
        set(_hako_mujoco_base_dir "${mujoco_precompiled_SOURCE_DIR}")
    endif()

    set(_hako_mujoco_include_dir "${_hako_mujoco_base_dir}/include")
    set(_hako_mujoco_import_lib "${_hako_mujoco_base_dir}/lib/mujoco.lib")
    set(_hako_mujoco_runtime_dll "${_hako_mujoco_base_dir}/bin/mujoco.dll")

    if(NOT EXISTS "${_hako_mujoco_include_dir}/mujoco/mujoco.h")
        message(FATAL_ERROR "MuJoCo headers not found: ${_hako_mujoco_include_dir}/mujoco/mujoco.h")
    endif()
    if(NOT EXISTS "${_hako_mujoco_import_lib}")
        message(FATAL_ERROR "MuJoCo import library not found: ${_hako_mujoco_import_lib}")
    endif()
    if(NOT EXISTS "${_hako_mujoco_runtime_dll}")
        message(FATAL_ERROR "MuJoCo runtime DLL not found: ${_hako_mujoco_runtime_dll}")
    endif()

    add_library(MuJoCo::mujoco SHARED IMPORTED GLOBAL)
    set_target_properties(MuJoCo::mujoco PROPERTIES
        IMPORTED_IMPLIB "${_hako_mujoco_import_lib}"
        IMPORTED_LOCATION "${_hako_mujoco_runtime_dll}"
        INTERFACE_INCLUDE_DIRECTORIES "${_hako_mujoco_include_dir}"
    )

    set(MUJOCO_SOURCE_DIR "${_hako_mujoco_include_dir}" PARENT_SCOPE)
    set(MUJOCO_LIB_DIR "${_hako_mujoco_base_dir}/lib" PARENT_SCOPE)
    set(MUJOCO_RUNTIME_DLL "${_hako_mujoco_runtime_dll}" PARENT_SCOPE)
    set(LIBMUJOCO "MuJoCo::mujoco" PARENT_SCOPE)
endfunction()

function(hako_configure_windows_runtime target_name)
    if(WIN32)
        set(_hako_runtime_dlls "")
        set(_hako_runtime_target_files "")
        if(NOT HAKO_USE_THIRDPARTY_HAKONIWA)
            if(EXISTS "${HAKONIWA_INSTALL_PREFIX}/bin")
                file(GLOB _hako_core_runtime_dlls "${HAKONIWA_INSTALL_PREFIX}/bin/*.dll")
                list(APPEND _hako_runtime_dlls ${_hako_core_runtime_dlls})
            endif()
            if(EXISTS "${HAKONIWA_PDU_ENDPOINT_PREFIX}/bin")
                file(GLOB _hako_endpoint_runtime_dlls "${HAKONIWA_PDU_ENDPOINT_PREFIX}/bin/*.dll")
                list(APPEND _hako_runtime_dlls ${_hako_endpoint_runtime_dlls})
            endif()
        endif()
        if(TARGET MuJoCo::mujoco)
            list(APPEND _hako_runtime_target_files "$<TARGET_FILE:MuJoCo::mujoco>")
        endif()
        list(REMOVE_DUPLICATES _hako_runtime_dlls)
        list(REMOVE_DUPLICATES _hako_runtime_target_files)
        if(_hako_runtime_dlls AND _hako_runtime_target_files)
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${target_name}>
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${_hako_runtime_target_files}
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${_hako_runtime_dlls}
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND_EXPAND_LISTS
            )
        elseif(_hako_runtime_dlls)
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${target_name}>
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${_hako_runtime_dlls}
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND_EXPAND_LISTS
            )
        elseif(_hako_runtime_target_files)
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${target_name}>
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${_hako_runtime_target_files}
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND_EXPAND_LISTS
            )
        else()
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${target_name}>
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND_EXPAND_LISTS
            )
        endif()
    endif()
endfunction()
if (DEFINED HAKO_CMAKE_OPTION_FILEPATH)
    message(STATUS "HAKO_CMAKE_OPTION_FILEPATH=${HAKO_CMAKE_OPTION_FILEPATH}")
    include(${HAKO_CMAKE_OPTION_FILEPATH})
else()
    message(STATUS "HAKO_CMAKE_OPTION_FILEPATH is not defined. Use default options.")
    set(HAKO_CMAKE_OPTION_FILEPATH "${PROJECT_SOURCE_DIR}/cmake-options/default-cmake-options.cmake")
    include(cmake-options/default-cmake-options.cmake)
endif()

set(PROJECT_ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
set(HAKO_THIRDPARTY_DIR "${PROJECT_ROOT_DIR}/thirdparty")
option(HAKO_USE_THIRDPARTY_HAKONIWA "Build against vendored Hakoniwa core/PDU endpoint under thirdparty/" OFF)

set(_hako_default_core_prefix "/usr/local/hakoniwa")
set(_hako_default_endpoint_prefix "")
if(DEFINED ENV{HAKONIWA_CORE_ROOT} AND NOT "$ENV{HAKONIWA_CORE_ROOT}" STREQUAL "")
    set(_hako_default_core_prefix "$ENV{HAKONIWA_CORE_ROOT}")
elseif(WIN32)
    if(EXISTS "${PROJECT_ROOT_DIR}/../hakoniwa-core-pro/install")
        set(_hako_default_core_prefix "${PROJECT_ROOT_DIR}/../hakoniwa-core-pro/install")
    endif()
endif()

if(DEFINED ENV{HAKONIWA_PDU_ENDPOINT_ROOT} AND NOT "$ENV{HAKONIWA_PDU_ENDPOINT_ROOT}" STREQUAL "")
    set(_hako_default_endpoint_prefix "$ENV{HAKONIWA_PDU_ENDPOINT_ROOT}")
elseif(WIN32)
    if(EXISTS "${PROJECT_ROOT_DIR}/../hakoniwa-pdu-endpoint/install")
        set(_hako_default_endpoint_prefix "${PROJECT_ROOT_DIR}/../hakoniwa-pdu-endpoint/install")
    endif()
endif()

set(HAKONIWA_INSTALL_PREFIX "${_hako_default_core_prefix}" CACHE PATH "Install prefix of hakoniwa-core-pro")
set(HAKONIWA_PDU_ENDPOINT_PREFIX "${_hako_default_endpoint_prefix}" CACHE PATH "Install prefix of hakoniwa-pdu-endpoint")
set(HAKONIWA_EXTRA_PREFIX_PATH "" CACHE STRING "Additional semicolon-separated prefixes for CMake package discovery")
set(MUJOCO_ROOT "" CACHE PATH "Manual MuJoCo root containing include/, lib/mujoco.lib, and bin/mujoco.dll")

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

        if(WIN32 AND MUJOCO_ROOT)
            set(MUJOCO_SOURCE_DIR "${MUJOCO_ROOT}/include")
            set(MUJOCO_LIB_DIR "${MUJOCO_ROOT}/lib")
            hako_define_windows_mujoco_target()
        else()
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
            if(WIN32)
                hako_define_windows_mujoco_target()
            else()
                set(LIBMUJOCO "mujoco")
            endif()
        endif()
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
    if(NOT WIN32)
        link_directories("${MUJOCO_LIB_DIR}")
    endif()
    message(STATUS "MuJoCo v${MUJOCO_VERSION} downloaded and configured.")
else()
    message(STATUS "MuJoCo disabled: Using simple physics model")
endif()

if(HAKO_USE_THIRDPARTY_HAKONIWA)
    set(HAKO_THIRDPARTY_CORE_DIR "${HAKO_THIRDPARTY_DIR}/hakoniwa-core-pro")
    set(HAKO_THIRDPARTY_PDU_ENDPOINT_DIR "${HAKO_THIRDPARTY_DIR}/hakoniwa-pdu-endpoint")
    if(NOT EXISTS "${HAKO_THIRDPARTY_CORE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "Vendored hakoniwa-core-pro not found: ${HAKO_THIRDPARTY_CORE_DIR}")
    endif()
    if(NOT EXISTS "${HAKO_THIRDPARTY_PDU_ENDPOINT_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "Vendored hakoniwa-pdu-endpoint not found: ${HAKO_THIRDPARTY_PDU_ENDPOINT_DIR}")
    endif()

    set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries instead of static libraries" FORCE)
    if(WIN32)
        set(HAKO_CLIENT_OPTION_FILEPATH "${HAKO_THIRDPARTY_CORE_DIR}/cmake-options/win-cmake-options.cmake" CACHE FILEPATH "" FORCE)
    endif()
    set(HAKO_CORE_PRO_BUILD_COMMAND OFF CACHE BOOL "" FORCE)
    set(HAKO_CORE_PRO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(HAKO_CORE_PRO_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
    set(HAKO_ENABLE_GTEST OFF CACHE BOOL "" FORCE)
    set(HAKO_PDU_ENDPOINT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(HAKO_PDU_ENDPOINT_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(HAKO_PDU_ENDPOINT_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(HAKO_PDU_ENDPOINT_ENABLE_HAKONIWA_CORE ON CACHE BOOL "" FORCE)
    set(HAKO_PDU_ENDPOINT_HAKONIWA_CORE_SOURCE_DIR "${HAKO_THIRDPARTY_CORE_DIR}" CACHE PATH "" FORCE)
    if(EXISTS "${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include/nlohmann/json.hpp")
        set(HAKO_LOCAL_NLOHMANN_JSON_ROOT "${PROJECT_ROOT_DIR}/thirdparty/nolman" CACHE PATH "" FORCE)
    endif()

    add_subdirectory("${HAKO_THIRDPARTY_CORE_DIR}" "${CMAKE_BINARY_DIR}/thirdparty/hakoniwa-core-pro" EXCLUDE_FROM_ALL)
    add_subdirectory("${HAKO_THIRDPARTY_PDU_ENDPOINT_DIR}" "${CMAKE_BINARY_DIR}/thirdparty/hakoniwa-pdu-endpoint" EXCLUDE_FROM_ALL)

    set(HAKO_CORE_INCLUDE_ROOT "${HAKO_THIRDPARTY_CORE_DIR}/include/hakoniwa")
    set(HAKO_CORE_INCLUDE_BASE "${HAKO_THIRDPARTY_CORE_DIR}/include")
    set(HAKO_PDU_TYPES_INCLUDE_DIR "${HAKO_THIRDPARTY_CORE_DIR}/hakoniwa-pdu-registry/pdu/types")
    set(HAKO_ASSETS_LINK_TARGET assets)
    set(HAKO_CONDUCTOR_LINK_TARGET conductor)
    set(HAKO_PDU_ENDPOINT_LINK_TARGET hakoniwa_pdu_endpoint::hakoniwa_pdu_endpoint)
    message(STATUS "Using vendored Hakoniwa targets from thirdparty/")
else()
    set(HAKO_CORE_INCLUDE_ROOT "${HAKONIWA_INSTALL_PREFIX}/include/hakoniwa")
    set(HAKO_CORE_INCLUDE_BASE "${HAKONIWA_INSTALL_PREFIX}/include")
    set(HAKO_PDU_TYPES_INCLUDE_DIR "${HAKONIWA_INSTALL_PREFIX}/include/hakoniwa/pdu")
    set(HAKO_CORE_LIBRARY_DIR "${HAKONIWA_INSTALL_PREFIX}/lib")

    set(_hako_package_prefixes "")
    foreach(_prefix IN ITEMS "${HAKONIWA_INSTALL_PREFIX}" "${HAKONIWA_PDU_ENDPOINT_PREFIX}")
        if(NOT "${_prefix}" STREQUAL "")
            list(APPEND _hako_package_prefixes "${_prefix}")
        endif()
    endforeach()
    if(NOT "${HAKONIWA_EXTRA_PREFIX_PATH}" STREQUAL "")
        list(APPEND _hako_package_prefixes ${HAKONIWA_EXTRA_PREFIX_PATH})
    endif()

    list(APPEND CMAKE_PREFIX_PATH ${_hako_package_prefixes})
    list(REMOVE_DUPLICATES CMAKE_PREFIX_PATH)

    message(STATUS "Hakoniwa core prefix: ${HAKONIWA_INSTALL_PREFIX}")
    if(NOT "${HAKONIWA_PDU_ENDPOINT_PREFIX}" STREQUAL "")
        message(STATUS "Hakoniwa PDU endpoint prefix: ${HAKONIWA_PDU_ENDPOINT_PREFIX}")
    endif()
    if(NOT "${HAKONIWA_EXTRA_PREFIX_PATH}" STREQUAL "")
        message(STATUS "Hakoniwa extra prefix path: ${HAKONIWA_EXTRA_PREFIX_PATH}")
    endif()

    find_package(hakoniwa-core CONFIG QUIET)
    find_package(hakoniwa_pdu_endpoint CONFIG QUIET)
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

    if(hakoniwa_pdu_endpoint_FOUND)
        set(HAKO_PDU_ENDPOINT_LINK_TARGET hakoniwa_pdu_endpoint::hakoniwa_pdu_endpoint)
    else()
        message(FATAL_ERROR
            "hakoniwa_pdu_endpoint package config not found.\n"
            "Checked CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}\n"
            "Set HAKONIWA_PDU_ENDPOINT_PREFIX to the install root that contains lib/cmake/hakoniwa_pdu_endpoint.\n"
            "Passing a build directory such as build-win or build-shared is not supported here; run cmake --install for hakoniwa-pdu-endpoint first."
        )
    endif()
endif()

option(USE_VIEWER "Enable MuJoCo viewer" ON)
MESSAGE(STATUS "USE_VIEWER=${USE_VIEWER}")
if(USE_VIEWER)
    MESSAGE(STATUS "MuJoCo viewer enabled")
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    add_library(mujoco-common STATIC mujoco-common/mujoco_viewer.cpp mujoco-common/mujoco_debug.cpp)
    target_include_directories(mujoco-common
        PUBLIC mujoco-common
        PUBLIC ${PROJECT_ROOT_DIR}/include
    )
    target_include_directories(mujoco-common SYSTEM PUBLIC "${MUJOCO_SOURCE_DIR}")
    hako_configure_target_warnings(mujoco-common)
    if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        target_link_libraries(mujoco-common PUBLIC ${OpenGL_LIBRARIES} "-framework OpenGL" glfw)
    else()
        target_link_libraries(mujoco-common PUBLIC OpenGL::GL glfw)
    endif()
else()
    MESSAGE(STATUS "MuJoCo viewer disabled")
    add_library(mujoco-common STATIC mujoco-common/mujoco_debug.cpp)
    target_include_directories(mujoco-common
        PUBLIC mujoco-common
        PUBLIC ${PROJECT_ROOT_DIR}/include
    )
    target_include_directories(mujoco-common SYSTEM PUBLIC "${MUJOCO_SOURCE_DIR}")
    hako_configure_target_warnings(mujoco-common)
endif()


add_subdirectory(sensors)
add_subdirectory(main_for_sample/forklift)
add_subdirectory(main_for_sample/tb3)
add_subdirectory(main_for_sample/drone_ball)
add_subdirectory(${PROJECT_ROOT_DIR}/examples ${CMAKE_BINARY_DIR}/examples)
