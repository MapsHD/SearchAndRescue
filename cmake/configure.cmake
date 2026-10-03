include_guard()

# ------------------------------------------------------------
# Build configuration
# ------------------------------------------------------------

if(CMAKE_CONFIGURATION_TYPES)
    set(HDM_SAR_CONFIGURE_IS_MULTICONFIG "ON")
    list(JOIN CMAKE_CONFIGURATION_TYPES ";" HDM_SAR_CONFIGURE_BUILD_TYPES)
else()
    set(HDM_SAR_CONFIGURE_IS_MULTICONFIG "OFF")
    set(HDM_SAR_CONFIGURE_BUILD_TYPES "${CMAKE_BUILD_TYPE}")
endif()

# ------------------------------------------------------------
# Platform
# ------------------------------------------------------------

set(HDM_SAR_CONFIGURE_SYSTEM            "${CMAKE_SYSTEM_NAME}")
set(HDM_SAR_CONFIGURE_SYSTEM_VERSION    "${CMAKE_SYSTEM_VERSION}")
set(HDM_SAR_CONFIGURE_CPU               "${CMAKE_SYSTEM_PROCESSOR}")

# ------------------------------------------------------------
# Compiler
# ------------------------------------------------------------

set(HDM_SAR_CONFIGURE_COMPILER          "${CMAKE_CXX_COMPILER_ID}")
set(HDM_SAR_CONFIGURE_COMPILER_VERSION  "${CMAKE_CXX_COMPILER_VERSION}")
set(HDM_SAR_CONFIGURE_COMPILER_PATH     "${CMAKE_CXX_COMPILER}")

# ------------------------------------------------------------
# CMake
# ------------------------------------------------------------

set(HDM_SAR_CONFIGURE_CMAKE_VERSION     "${CMAKE_VERSION}")
set(HDM_SAR_CONFIGURE_GENERATOR         "${CMAKE_GENERATOR}")

# ------------------------------------------------------------
# Project
# ------------------------------------------------------------

set(HDM_SAR_CONFIGURE_PROJECT_NAME      "${PROJECT_NAME}")
set(HDM_SAR_CONFIGURE_PROJECT_VERSION   "${PROJECT_VERSION}")

# ------------------------------------------------------------
# Language
# ------------------------------------------------------------

set(HDM_SAR_CONFIGURE_CXX_STANDARD      "${CMAKE_CXX_STANDARD}")

# ------------------------------------------------------------
# Timestamp
# ------------------------------------------------------------

string(TIMESTAMP HDM_SAR_CONFIGURE_TIMESTAMP "%Y-%m-%d %H:%M:%S")

# ------------------------------------------------------------
# Git
# ------------------------------------------------------------

execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE HDM_SAR_CONFIGURE_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE HDM_SAR_CONFIGURE_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

foreach(VAR
    HDM_SAR_CONFIGURE_GIT_HASH
    HDM_SAR_CONFIGURE_GIT_BRANCH)
    if(NOT ${VAR})
        set(${VAR} "unknown")
    endif()
endforeach()

configure_file(
    "${REPOSITORY_DIRECTORY}/cmake/configure/configure.hpp.in"
    "${REPOSITORY_DIRECTORY}/generated/HDM_SAR_ConfigureInfo.hpp"
    @ONLY
)