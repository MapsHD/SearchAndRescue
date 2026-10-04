include_guard()

set(IMGUI_SOURCE_FILES
    ${EXTERNAL_DIRECTORY}/imgui/imgui.cpp
    ${EXTERNAL_DIRECTORY}/imgui/imgui_draw.cpp
    ${EXTERNAL_DIRECTORY}/imgui/imgui_tables.cpp
    ${EXTERNAL_DIRECTORY}/imgui/imgui_widgets.cpp
    ${EXTERNAL_DIRECTORY}/imgui/backends/imgui_impl_opengl3.cpp
    ${EXTERNAL_DIRECTORY}/imgui/backends/imgui_impl_glfw.cpp
)

set(IMGUI_HEADER_FILES
    ${EXTERNAL_DIRECTORY}/imgui/imgui.h
    ${EXTERNAL_DIRECTORY}/imgui/backends/imgui_impl_opengl3.h
    ${EXTERNAL_DIRECTORY}/imgui/backends/imgui_impl_glfw.h
)

add_library(imgui
    STATIC
)

target_sources(imgui
    PRIVATE
        ${IMGUI_SOURCE_FILES}
        ${IMGUI_HEADER_FILES}
)

target_include_directories(imgui
    PUBLIC
        ${EXTERNAL_DIRECTORY}/glfw/include
        ${EXTERNAL_DIRECTORY}/imgui
        ${EXTERNAL_DIRECTORY}/imgui/backends
)
