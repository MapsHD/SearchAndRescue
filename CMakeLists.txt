cmake_minimum_required(VERSION 3.20.0 FATAL_ERROR)

project(HDMapping-SearchAndRescue VERSION 1.0.4)

set(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
set(EXTERNAL_DIRECTORY ${REPOSITORY_DIRECTORY}/external)

# Project version : exposed to C++ as CAVE_TRAVERSAL_TOOL_VERSION_MAJOR / _MINOR / _PATCH defines
add_compile_definitions(
    CAVE_TRAVERSAL_TOOL_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    CAVE_TRAVERSAL_TOOL_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    CAVE_TRAVERSAL_TOOL_VERSION_PATCH=${PROJECT_VERSION_PATCH})

set(CMAKE_CXX_STANDARD            23)
set(CMAKE_CXX_STANDARD_REQUIRED   ON)
set(CMAKE_CXX_EXTENSIONS          OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_definitions(GLM_FORCE_DEPTH_ZERO_TO_ONE)
add_compile_definitions(GLM_FORCE_Z_CLIP_CONTROL)
add_compile_definitions(GLM_FORCE_RIGHT_HANDED)
add_compile_definitions(GLM_FORCE_RADIANS)
add_compile_definitions(GLM_FORCE_XYZW_ONLY)
add_compile_definitions(GLM_FORCE_QUAT_DATA_XYZW)
add_compile_definitions(GLM_FORCE_QUAT_CTOR_XYZW)

if(MSVC)
    add_compile_definitions(NOMINMAX)
    add_compile_definitions(WIN32_LEAN_AND_MEAN)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

if (MSVC)
  add_compile_options(/utf-8)
endif()

set(GLFW_USE_HYBRID_HPG    ON CACHE BOOL "" FORCE)
set(GLFW_BUILD_WAYLAND     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_X11         ON  CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS        OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS       OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES    OFF CACHE BOOL "" FORCE)

set(SPDLOG_HEADER_ONLY     OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_SHARED    OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS     OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH     OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLE   OFF CACHE BOOL "" FORCE)

set(LASZIP_BUILD_STATIC    ON  CACHE BOOL "" FORCE)

add_subdirectory(${EXTERNAL_DIRECTORY}/glfw)
add_subdirectory(${EXTERNAL_DIRECTORY}/spdlog)
add_subdirectory(${EXTERNAL_DIRECTORY}/LASzip)

include(${REPOSITORY_DIRECTORY}/cmake/glad.cmake)
include(${REPOSITORY_DIRECTORY}/cmake/imgui.cmake)
include(${REPOSITORY_DIRECTORY}/cmake/imguizmo.cmake)
include(${REPOSITORY_DIRECTORY}/cmake/implot.cmake)

add_subdirectory(lib/Core)
add_subdirectory(app)
