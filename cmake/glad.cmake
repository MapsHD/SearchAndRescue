include_guard()

set(GLAD_SOURCE_FILES
    ${EXTERNAL_DIRECTORY}/glad/src/glad.c
)

set(GLAD_HEADER_FILES
    ${EXTERNAL_DIRECTORY}/glad/include/glad/glad.h
    ${EXTERNAL_DIRECTORY}/glad/include/KHR/khrplatform.h
)

add_library(glad
    STATIC
)

target_sources(glad
    PRIVATE
        ${GLAD_SOURCE_FILES}
        ${GLAD_HEADER_FILES}
)

target_include_directories(glad
    PUBLIC
        ${EXTERNAL_DIRECTORY}/glad/include
)
