include_guard()

set(IMGUIZMO_SOURCE_FILES
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/GraphEditor.cpp
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImCurveEdit.cpp
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImGradient.cpp
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImGuizmo.cpp
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImSequencer.cpp
)

set(IMGUIZMO_HEADER_FILES
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/GraphEditor.h
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImCurveEdit.h
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImGradient.h
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImGuizmo.h
    ${EXTERNAL_DIRECTORY}/ImGuizmo/src/ImSequencer.h
)

add_library(imguizmo
    STATIC
)

target_sources(imguizmo
    PRIVATE
        ${IMGUIZMO_SOURCE_FILES}
        ${IMGUIZMO_HEADER_FILES}
)

target_include_directories(imguizmo
    PUBLIC
        ${EXTERNAL_DIRECTORY}/glfw/include
        ${EXTERNAL_DIRECTORY}/imgui
        ${EXTERNAL_DIRECTORY}/imgui/backends
        ${EXTERNAL_DIRECTORY}/ImGuizmo/src
)
