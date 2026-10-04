#pragma once

#include <string>

// Application version as configured in the root CMakeLists.txt
// (project(... VERSION x.y.z) -> compile definitions)
inline std::string application_version()
{
    return std::to_string(HDMAPPING_REARCH_AND_RESCUE_VERSION_MAJOR) + "." +
           std::to_string(HDMAPPING_REARCH_AND_RESCUE_VERSION_MINOR) + "." +
           std::to_string(HDMAPPING_REARCH_AND_RESCUE_VERSION_PATCH);
}
