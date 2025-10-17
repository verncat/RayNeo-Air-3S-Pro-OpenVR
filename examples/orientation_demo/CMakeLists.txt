cmake_minimum_required(VERSION 3.10)
project(RayNeoOrientationDemo)

# Demo executable -------------------------------------------------------------
set(SRC main.cpp)
add_executable(RayNeoOrientationDemo ${SRC})
target_include_directories(RayNeoOrientationDemo PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_dependencies(RayNeoOrientationDemo RayNeoSDK)
target_link_libraries(RayNeoOrientationDemo PRIVATE RayNeoSDK)

# Core rendering dependencies -------------------------------------------------
find_package(OpenGL REQUIRED)
target_link_libraries(RayNeoOrientationDemo PRIVATE OpenGL::GL)

find_package(SDL2 CONFIG REQUIRED)
target_link_libraries(RayNeoOrientationDemo
    PRIVATE
        $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
        $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
)
target_compile_definitions(RayNeoOrientationDemo PRIVATE SDL_MAIN_HANDLED)

find_package(SDL2_ttf CONFIG REQUIRED)
target_link_libraries(RayNeoOrientationDemo PRIVATE
    $<IF:$<TARGET_EXISTS:SDL2_ttf::SDL2_ttf>,SDL2_ttf::SDL2_ttf,SDL2_ttf::SDL2_ttf-static>
)

# Windows runtime deployment --------------------------------------------------
rayneo_deploy_runtime(RayNeoOrientationDemo)

# RPATH so the executable finds libRayNeoSDK when in same directory
if (APPLE)
    set_target_properties(RayNeoOrientationDemo PROPERTIES BUILD_RPATH "@loader_path" INSTALL_RPATH "@loader_path")
else()
    set_target_properties(RayNeoOrientationDemo PROPERTIES BUILD_RPATH "$ORIGIN" INSTALL_RPATH "$ORIGIN")
endif()
