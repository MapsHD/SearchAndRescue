#include <Core/ErrorCallbacks.h>

#include <spdlog/spdlog.h>

namespace ErrorCallback
{
    void GLFW(const int32_t code, const char* message)
    {
        spdlog::error("GLFW error {} : {}", code, message);
    }
}