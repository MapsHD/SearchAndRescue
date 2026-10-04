cmake_minimum_required(VERSION 3.20.0 FATAL_ERROR)

add_executable(HDMapping-SearchAndRescue main.cpp)

target_link_libraries(HDMapping-SearchAndRescue
    PRIVATE
        Core
)

target_include_directories(HDMapping-SearchAndRescue
    PRIVATE
        ${REPOSITORY_DIRECTORY}/generated
)
