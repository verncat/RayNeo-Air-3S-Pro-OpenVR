cmake_minimum_required(VERSION 3.10)

# Simple stub OpenVR driver library
project(OpenVRStubDriver LANGUAGES CXX)


set(SIZEOF_VOIDP ${CMAKE_SIZEOF_VOID_P})
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROCESSOR_ARCH "64")
else()
    set(PROCESSOR_ARCH "32")
endif()

# Force 64-bit build
set(PROCESSOR_ARCH "64")

if(WIN32)
    set(PLATFORM_NAME "win")
elseif(UNIX AND NOT APPLE)
    set(PLATFORM_NAME "linux")
elseif(APPLE)
    set(PLATFORM_NAME "osx")
endif()

message("Building for platform: ${PLATFORM_NAME}${PROCESSOR_ARCH}")

file(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(openvr_stub_driver SHARED
    ${HEADERS}
    ${SOURCES}
    )

# install(
#   FILES $<TARGET_RUNTIME_DLLS:openvr_stub_driver>
#   DESTINATION ${CMAKE_INSTALL_LIBDIR}
# )

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "Header Files" FILES ${HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "Source Files" FILES ${SOURCES})

target_include_directories(openvr_stub_driver
    PRIVATE
        ${CMAKE_SOURCE_DIR}/thirdparties/openvr/headers
        ${CMAKE_SOURCE_DIR}/openvr_driver
        ${CMAKE_SOURCE_DIR}/include
)

target_compile_features(openvr_stub_driver PRIVATE cxx_std_17)
target_compile_options(openvr_stub_driver PRIVATE -fPIC)


# On macOS OpenVR expects a dylib; ensure correct suffix if needed
set_target_properties(openvr_stub_driver PROPERTIES OUTPUT_NAME "driver_stub")
set_target_properties(openvr_stub_driver PROPERTIES PREFIX "driver_")
set_target_properties(openvr_stub_driver PROPERTIES OUTPUT_NAME "rayneo")

# Platform-specific output directory (match SteamVR driver folder conventions)
if (WIN32)
    set(_DRV_OUT_DIR ${PROJECT_BINARY_DIR}/bin/win64)
elseif(UNIX AND NOT APPLE)
    set(_DRV_OUT_DIR ${PROJECT_BINARY_DIR}/bin/linux64)
elseif(APPLE)
    # macOS still often uses osx64 naming in SteamVR
    set(_DRV_OUT_DIR ${PROJECT_BINARY_DIR}/bin/osx64)
endif()
if (NOT _DRV_OUT_DIR)
    set(_DRV_OUT_DIR ${PROJECT_BINARY_DIR}/bin/win64) # fallback
endif()
set_target_properties(openvr_stub_driver PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${_DRV_OUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${_DRV_OUT_DIR}
)

if (TARGET RayNeoSDK)
    target_link_libraries(openvr_stub_driver PRIVATE RayNeoSDK)
    # Deploy RayNeo runtime next to driver using central helper from top-level CMake.
    if (COMMAND rayneo_deploy_runtime)
        rayneo_deploy_runtime(openvr_stub_driver)
    else()
        message(WARNING "rayneo_deploy_runtime() not available; runtime DLLs may not be copied.")
    endif()
else()
    message(FATAL_ERROR "RayNeoSDK target not found; build top-level project with RAYNEO_BUILD_OPENVR_DRIVER=ON")
endif()


install(TARGETS openvr_stub_driver
        LIBRARY DESTINATION lib)

# Install manifest (for SteamVR place under drivers/stub/driver.vrdrivermanifest)
configure_file(driver.vrdrivermanifest ${CMAKE_CURRENT_BINARY_DIR}/driver.vrdrivermanifest COPYONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/driver.vrdrivermanifest DESTINATION drivers/stub) # typical SteamVR layout suggestion
install(TARGETS openvr_stub_driver LIBRARY DESTINATION drivers/stub/bin) # place binary under bin

# ---------------------------------------------------------------------------
# Optional deploy into a local SteamVR drivers directory (Windows)
# Usage:
#   cmake --build . --target deploy_openvr_driver
# You can override by: -DSTEAMVR_DRIVERS_DIR="C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers"
# ---------------------------------------------------------------------------
if (WIN32)
    # Attempt to auto-discover SteamVR install path from registry (Steam App 250820)
    # Registry key: HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Steam App 250820\InstallLocation
    set(_STEAMVR_REG_KEY "HKLM\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Steam App 250820")
    set(_STEAMVR_REG_VALUE "InstallLocation")
    set(_STEAMVR_INSTALL_LOCATION "")
    if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
        # CMake 3.24+ has built-in registry query via generator expressions (not for general variables). Fallback to powershell.
    endif()
    if (NOT _STEAMVR_INSTALL_LOCATION)
        execute_process(
            COMMAND powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -Command "(Get-ItemProperty -Path 'Registry::${_STEAMVR_REG_KEY}' -ErrorAction SilentlyContinue).${_STEAMVR_REG_VALUE}"
            OUTPUT_VARIABLE _STEAMVR_INSTALL_LOCATION_RAW
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        string(STRIP "${_STEAMVR_INSTALL_LOCATION_RAW}" _STEAMVR_INSTALL_LOCATION)
    endif()
    
    if (_STEAMVR_INSTALL_LOCATION)
        message(STATUS "Detected SteamVR InstallLocation: ${_STEAMVR_INSTALL_LOCATION}")
        set(DEFAULT_STEAMVR_DIR "${_STEAMVR_INSTALL_LOCATION}/drivers")
    elseif(DEFAULT_STEAMVR_DIR)
        message(STATUS "SteamVR registry key not found; using provided: ${DEFAULT_STEAMVR_DIR}")
    endif()

    if(DEFAULT_STEAMVR_DIR)
        set(STEAMVR_DRIVERS_DIR "${DEFAULT_STEAMVR_DIR}" CACHE PATH "Path to SteamVR drivers directory")
        set(DRIVER_NAME "openvr_driver" CACHE STRING "Name of the driver folder under SteamVR/drivers")

        add_custom_target(deploy_openvr_driver
            COMMAND ${CMAKE_COMMAND} -E echo "[deploy] Deploying OpenVR driver to ${STEAMVR_DRIVERS_DIR}/${DRIVER_NAME}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${STEAMVR_DRIVERS_DIR}/${DRIVER_NAME}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:openvr_stub_driver>" "${STEAMVR_DRIVERS_DIR}/${DRIVER_NAME}/bin/${PLATFORM_NAME}${PROCESSOR_ARCH}/$<TARGET_FILE_NAME:openvr_stub_driver>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${CMAKE_CURRENT_BINARY_DIR}/driver.vrdrivermanifest" "${STEAMVR_DRIVERS_DIR}/${DRIVER_NAME}/driver.vrdrivermanifest"
            COMMAND ${CMAKE_COMMAND} -E copy -t "${STEAMVR_DRIVERS_DIR}/${DRIVER_NAME}/bin/${PLATFORM_NAME}${PROCESSOR_ARCH}/" $<TARGET_RUNTIME_DLLS:openvr_stub_driver>
            COMMAND_EXPAND_LISTS
            COMMAND ${CMAKE_COMMAND} -E echo "[deploy] Done."
            DEPENDS openvr_stub_driver
        )
    else()
        message(WARNING "!!! SteamVR InstallLocation registry key not found; please set STEAMVR_DRIVERS_DIR manually. Skipping deploy target.")
    endif()
endif()
