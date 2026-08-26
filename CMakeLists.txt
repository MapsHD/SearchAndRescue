cmake_minimum_required(VERSION 3.20.0 FATAL_ERROR)

project(cave-traversal-tool)

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

set(REPOSITORY_DIR ${CMAKE_CURRENT_LIST_DIR})
set(SHADERS_DIR ${REPOSITORY_DIR}/assets) 

message("Shaders directory : ${SHADERS_DIR}") 

set(BUILD_SHARED_LIBS    OFF CACHE BOOL "" FORCE)

set(GLFW_BUILD_WAYLAND   OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_X11       ON  CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS      OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES  OFF CACHE BOOL "" FORCE)

set(SPDLOG_HEADER_ONLY   ON  CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_SHARED  OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH   OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)

set(LASZIP_BUILD_STATIC  ON  CACHE BOOL "" FORCE)

add_subdirectory(external/glfw)
add_subdirectory(external/spdlog)
add_subdirectory(external/LASzip)

add_executable(cave-traversal-tool main.cpp)

target_sources(cave-traversal-tool
    PRIVATE
        external/glad/src/glad.c
        src/Buffer.cpp
        src/Camera.cpp
        src/ErrorCallbacks.cpp
        src/FileIO.cpp
        src/GPUData.cpp
        src/Helpers.cpp
        src/Processing.cpp
        src/Program.cpp
        src/VertexArray.cpp
        from_hdmapping/pair_wise_iterative_closest_point.cpp
        from_hdmapping/hash_utils.cpp
    PRIVATE
        external/glad/src/glad.c
        external/imgui/imgui_demo.cpp
        external/imgui/imgui_draw.cpp
        external/imgui/imgui_tables.cpp
        external/imgui/imgui_widgets.cpp
        external/imgui/imgui.cpp
        external/imgui/backends/imgui_impl_glfw.cpp
        external/imgui/backends/imgui_impl_opengl3.cpp
        external/ImGuizmo/src/GraphEditor.cpp
        external/ImGuizmo/src/ImCurveEdit.cpp
        external/ImGuizmo/src/ImGradient.cpp
        external/ImGuizmo/src/ImGuizmo.cpp
        external/ImGuizmo/src/ImSequencer.cpp)

target_link_libraries(cave-traversal-tool
    PRIVATE
        glfw
        laszip3)

target_include_directories(cave-traversal-tool
    PRIVATE
        include
    PRIVATE
        external/eigen
        external/glfw/include
        external/glad/include
        external/glm
        external/imgui
        external/imgui/backends
        external/ImGuizmo/src
        external/portable-file-dialogs
        external/happly
        external/spdlog/include
        external/LASzip/dll
        external/LASzip/include/laszip)

target_compile_definitions(cave-traversal-tool
    PRIVATE
        -DSHADERS_DIR="${SHADERS_DIR}")
