include_guard()

set(IMPLOT_SOURCE_FILES
    ${EXTERNAL_DIRECTORY}/implot/implot_items.cpp
    ${EXTERNAL_DIRECTORY}/implot/implot.cpp
    # DEMO
    # ${EXTERNAL_DIRECTORY}/implot/implot_demo.cpp
)

set(IMPLOT_HEADER_FILES
    ${EXTERNAL_DIRECTORY}/implot/implot_internal.h
    ${EXTERNAL_DIRECTORY}/implot/implot.h
)

add_library(implot
    STATIC
)

target_sources(implot
    PRIVATE
        ${IMPLOT_SOURCE_FILES}
        ${IMPLOT_HEADER_FILES}
)

target_include_directories(implot
    PUBLIC
        ${EXTERNAL_DIRECTORY}/implot
        ${EXTERNAL_DIRECTORY}/imgui
        ${EXTERNAL_DIRECTORY}/imgui/backends
)
