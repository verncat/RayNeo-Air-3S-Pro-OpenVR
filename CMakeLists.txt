cmake_minimum_required(VERSION 3.16)

project(RayNeo VERSION 1.0 LANGUAGES C CXX)

# Enable modern RPATH behavior on macOS
set(CMAKE_MACOSX_RPATH ON)

# User-facing build toggles ---------------------------------------------------
option(RAYNEO_BUILD_OPENVR_DRIVER "Build the OpenVR stub driver" ON)
option(RAYNEO_BUILD_EXAMPLES "Build example applications" ON)

# -----------------------------------------------------------------------------
# Runtime Deployment Helper
# We centralize the logic that copies the RayNeoSDK shared library and (on
# non-Apple platforms) the libusb runtime next to any executable target that
# uses the SDK. This avoids duplication across examples and future tools.
# Usage: rayneo_deploy_runtime(<target>) after the target is defined.
# -----------------------------------------------------------------------------

function(rayneo_deploy_runtime tgt)
    if (NOT TARGET ${tgt})
        message(FATAL_ERROR "rayneo_deploy_runtime called with non-existent target '${tgt}'")
    endif()
    if (NOT TARGET RayNeoSDK)
        message(FATAL_ERROR "RayNeoSDK target not found; deploy helper requires it")
    endif()

    # Always copy RayNeoSDK shared lib (DLL/.dylib/.so) to the target dir.
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:RayNeoSDK>
            $<TARGET_FILE_DIR:${tgt}>
        COMMENT "Deploying RayNeoSDK runtime to ${tgt}")

    # Non-Apple: attempt libusb runtime copy if path known.
    if (NOT APPLE AND RAYNEO_LIBUSB_RUNTIME_PATH)
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${RAYNEO_LIBUSB_RUNTIME_PATH}" $<TARGET_FILE_DIR:${tgt}>/libusb-1.0.dll
            COMMENT "Deploying libusb runtime to ${tgt}")
    endif()
endfunction()

# Detect libusb using pkg-config first, falling back to manual search ----------
find_package(PkgConfig QUIET)
if (PkgConfig_FOUND)
    pkg_check_modules(Libusb QUIET IMPORTED_TARGET libusb-1.0)
endif()

# Provide a unified interface target that downstream code can link against
add_library(LibUsb INTERFACE)
add_library(RayNeo::LibUsb ALIAS LibUsb)

if (TARGET PkgConfig::libusb)
    # pkg-config found: reuse its imported target and propagate its compile flags
    target_link_libraries(LibUsb INTERFACE PkgConfig::libusb)
    if (Libusb_CFLAGS_OTHER)
        target_compile_options(LibUsb INTERFACE ${Libusb_CFLAGS_OTHER})
    endif()
else()
    # Fallback: locate headers and libraries manually
    find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES libusb-1.0)
    find_library(LIBUSB_LIBRARY NAMES usb-1.0 libusb-1.0)
    if (NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
        message(FATAL_ERROR "libusb-1.0 not found. Install libusb or provide pkg-config file.")
    endif()
    target_include_directories(LibUsb INTERFACE ${LIBUSB_INCLUDE_DIR})
    target_link_libraries(LibUsb INTERFACE ${LIBUSB_LIBRARY})
endif()

# Derive a libusb runtime path for deployment on non-Apple systems (Windows, Linux).
# Windows wants the DLL; Linux can typically rely on system loader (so copy optional).
if (NOT APPLE)
    set(RAYNEO_LIBUSB_RUNTIME_PATH "")
    if (WIN32)
        # Prefer pkg-config prefix bin directory if available.
        if (TARGET PkgConfig::libusb AND Libusb_PREFIX)
            set(_rayneo_candidate "${Libusb_PREFIX}/bin/libusb-1.0.dll")
            if (EXISTS "${_rayneo_candidate}")
                set(RAYNEO_LIBUSB_RUNTIME_PATH "${_rayneo_candidate}")
            endif()
        endif()
        # Fallback to vcpkg conventional location.
        if (RAYNEO_LIBUSB_RUNTIME_PATH STREQUAL "" AND DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
            set(_rayneo_vcpkg_candidate "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin/libusb-1.0.dll")
            if (EXISTS "${_rayneo_vcpkg_candidate}")
                set(RAYNEO_LIBUSB_RUNTIME_PATH "${_rayneo_vcpkg_candidate}")
            endif()
        endif()
        if (RAYNEO_LIBUSB_RUNTIME_PATH STREQUAL "")
            message(WARNING "Could not determine libusb-1.0.dll path for deployment; ensure DLL is on PATH or copied manually")
        else()
            message(STATUS "libusb runtime resolved: ${RAYNEO_LIBUSB_RUNTIME_PATH}")
        endif()
    else()
        # On Linux we usually rely on system loader; attempt to find .so for optional copy.
    get_target_property(_rayneo_libusb_lib LibUsb INTERFACE_LINK_LIBRARIES)
        # _rayneo_libusb_lib may list several; attempt to locate a libusb shared object hint.
        foreach(lib IN LISTS _rayneo_libusb_lib)
            if (lib MATCHES "libusb" AND EXISTS "${lib}")
                set(RAYNEO_LIBUSB_RUNTIME_PATH "${lib}")
                break()
            endif()
        endforeach()
        if (RAYNEO_LIBUSB_RUNTIME_PATH)
            message(STATUS "(Optional) libusb runtime candidate: ${RAYNEO_LIBUSB_RUNTIME_PATH}")
        endif()
    endif()
endif()

# OpenGL policy to prefer GLVND when available (silences warning)
if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()

# Platform-specific dependencies ----------------------------------------------
if (APPLE)
    set(RAYNEO_PLATFORM_LIBS "-framework IOKit" "-framework CoreFoundation")
else()
    find_package(Threads REQUIRED)
    set(RAYNEO_PLATFORM_LIBS Threads::Threads)
endif()

# RayNeo C API shared library -------------------------------------------------
add_library(RayNeoSDK SHARED
    src/RayneoApi.cpp
)

target_include_directories(RayNeoSDK
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(RayNeoSDK
    PRIVATE
        RayNeo::LibUsb
        ${RAYNEO_PLATFORM_LIBS}
)

# ----------------------------------------------------------------------------
# RPATH Setup: ensure executables and the SDK can locate libRayNeoSDK.* when
# placed side-by-side (CI packaging strategy). We set BUILD_RPATH so running
# from build tree works, and INSTALL_RPATH for later installation or archives.
# Linux: use $ORIGIN ; macOS: use @loader_path
# ----------------------------------------------------------------------------
if (APPLE)
    set(_RAYNEO_RPATH "@loader_path")
else()
    set(_RAYNEO_RPATH "$ORIGIN")
endif()

set_target_properties(RayNeoSDK PROPERTIES
    BUILD_RPATH "${_RAYNEO_RPATH}"
    INSTALL_RPATH "${_RAYNEO_RPATH}"
)

if (WIN32)
    # Deploy libusb alongside the SDK itself (primary location).
    if (RAYNEO_LIBUSB_RUNTIME_PATH)
        add_custom_command(TARGET RayNeoSDK POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${RAYNEO_LIBUSB_RUNTIME_PATH}" "$<TARGET_FILE_DIR:RayNeoSDK>/libusb-1.0.dll"
            COMMENT "Deploying libusb next to RayNeoSDK.dll")
    endif()
endif()

# Optional components ----------------------------------------------------------
if (RAYNEO_BUILD_OPENVR_DRIVER)
    add_subdirectory(openvr_driver)
endif()

if (RAYNEO_BUILD_EXAMPLES)
    add_subdirectory(examples/simple)
    add_subdirectory(examples/orientation_demo)
endif()