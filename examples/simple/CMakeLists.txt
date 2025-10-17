cmake_minimum_required(VERSION 3.10)
project(RayNeoSimpleExample CXX)

# Sample console program ------------------------------------------------------
add_executable(RayNeoExample main.cpp)

# Ensure the SDK target is available from the parent project; fail otherwise
if (TARGET RayNeoSDK)
    target_link_libraries(RayNeoExample PRIVATE RayNeoSDK)
else()
    message(FATAL_ERROR "RayNeoSDK target not found. Add this directory from the main build.")
endif()

# Expose public SDK headers (helpful even for standalone builds)
target_include_directories(RayNeoExample PRIVATE ${CMAKE_SOURCE_DIR}/include)

# On non-macOS platforms we may need to link libusb manually ------------------
if (NOT APPLE)
    # Libusb headers/libs are already propagated transitively via RayNeoSDK's
    # dependency on RayNeo::LibUsb; no need to repeat discovery here.
endif()

# Post-build deployment helpers (Windows) -------------------------------------
rayneo_deploy_runtime(RayNeoExample)

# RPATH so the executable finds libRayNeoSDK when in same directory
if (APPLE)
    set_target_properties(RayNeoExample PROPERTIES BUILD_RPATH "@loader_path" INSTALL_RPATH "@loader_path")
else()
    set_target_properties(RayNeoExample PROPERTIES BUILD_RPATH "$ORIGIN" INSTALL_RPATH "$ORIGIN")
endif()
