#include <cave-traversal-tool/OpenGL/Program.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

#include <string>
#include <vector>

uint32_t opengl_program_compile_shader(const std::vector<char>& source, uint32_t shader_type)
{
    std::string src_string(source.begin(), source.end());
    const char* src_ptr = src_string.c_str();

    uint32_t shader_id = glCreateShader(shader_type);
    if (shader_id == 0)
    {
        spdlog::error("Failed to create shader object!");
        std::abort();
    }

    glShaderSource(shader_id, 1, &src_ptr, nullptr);
    glCompileShader(shader_id);

    int32_t compile_status = 0;
    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &compile_status);

    if (compile_status != 1)
    {
        int32_t log_length = 0;
        glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_length);

        std::string log(log_length, '\0');
        glGetShaderInfoLog(shader_id, log_length, nullptr, log.data());

        spdlog::error("Shader compilation failed : {}", log.c_str());

        glDeleteShader(shader_id);
        return 0;
    }

    return shader_id;
}

uint32_t opengl_program_create_program(uint32_t vertex_shader_id, uint32_t fragment_shader_id)
{
    uint32_t program_id = glCreateProgram();
    if (program_id == 0)
    {
        spdlog::error("Failed to create program object!");
        std::abort();
    }

    glAttachShader(program_id, vertex_shader_id);
    glAttachShader(program_id, fragment_shader_id);
    glLinkProgram(program_id);

    int32_t link_status = 0;
    glGetProgramiv(program_id, GL_LINK_STATUS, &link_status);

    if (link_status != 1)
    {
        int32_t log_length = 0;
        glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &log_length);

        std::string log(log_length, '\0');
        glGetProgramInfoLog(program_id, log_length, nullptr, log.data());

        spdlog::error("Program linking failed : {}", log.c_str());

        glDeleteProgram(program_id);
        return 0;
    }

    glDetachShader(program_id, vertex_shader_id);
    glDetachShader(program_id, fragment_shader_id);

    return program_id;
}