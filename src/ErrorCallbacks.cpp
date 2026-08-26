#include <cave-traversal-tool/ErrorCallbacks.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

namespace ErrorCallback
{
    void GLFW(const int32_t code, const char* message)
    {
        spdlog::error("GLFW error {} : {}", code, message);
    }

    void OpenGL(const uint32_t source, const uint32_t type, const uint32_t id, const uint32_t severity, const int32_t length, const char* message, const void* user_data)
    {
        switch (severity)
        {
        case GL_DEBUG_SEVERITY_HIGH:
            {
                spdlog::error("OpenGL error {} : {}", id, message);
                break;
            }
        case GL_DEBUG_SEVERITY_MEDIUM:
            {
                spdlog::warn("OpenGL warning {} : {}", id, message);
                break;
            }
        case GL_DEBUG_SEVERITY_LOW:
            {
                spdlog::info("OpenGL info {} : {}", id, message);
                break;
            }
        default:
            {
                break;
            }
        }
    }
}