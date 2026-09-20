#include <cave-traversal-tool/OpenGL/Program.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

#include <glm/gtc/type_ptr.hpp>

#include <unordered_map>

struct Program::ProgramIMPL
{
    uint32_t id = {};

    std::unordered_map<std::string, int32_t> uniform_location_cache = {};
};

static constexpr int32_t kInvalidLocation = -1;

Program::Program(const std::vector<ShaderDescriptor>& program_descriptor)
{
    _impl = new ProgramIMPL;

    _impl->id                     = UINT32_MAX;
    _impl->uniform_location_cache = {};

    std::vector<uint32_t> shaders = {};
    for (const ShaderDescriptor& shader_descriptor : program_descriptor)
    {
        uint32_t shader = glCreateShader(shader_descriptor.shader_type);

        glShaderSource(shader, 1, &shader_descriptor.source, &shader_descriptor.source_size);
        glCompileShader(shader);

        int32_t compilation_status = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compilation_status);

        if (!compilation_status)
        {
            int32_t info_log_length = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_log_length);

            std::vector<char> info_log(info_log_length);
            glGetShaderInfoLog(shader, info_log_length, &info_log_length, info_log.data());

            spdlog::error("Failed to validate shader compilation: {}", info_log.data());
        }

        shaders.push_back(shader);
    }

    _impl->id = glCreateProgram();

    for (const uint32_t shader : shaders)
    {
        glAttachShader(_impl->id, shader);
    }

    glLinkProgram(_impl->id);

    int32_t link_status = GL_FALSE;
    glGetProgramiv(_impl->id, GL_LINK_STATUS, &link_status);

    if (!link_status)
    {
        int32_t info_log_length = 0;
        glGetProgramiv(_impl->id, GL_INFO_LOG_LENGTH, &info_log_length);

        std::vector<char> info_log(info_log_length);
        glGetProgramInfoLog(_impl->id, info_log_length, &info_log_length, info_log.data());

        spdlog::error("Failed to validate program linking: {}", info_log.data());
    }

    for (const uint32_t shader : shaders)
    {
        glDetachShader(_impl->id, shader);
        glDeleteShader(shader);
    }

    int32_t uniform_count = 0;
    glGetProgramiv(_impl->id, GL_ACTIVE_UNIFORMS, &uniform_count);

    spdlog::debug("Active uniforms {}", uniform_count);

    for (int32_t uniform_index = 0; uniform_index < uniform_count; uniform_index++)
    {
        int32_t  size = 0;
        uint32_t type = 0;

        const int32_t buffer_size       = 128;
        char          name[buffer_size] = {};
        int32_t       length            = 0;

        glGetActiveUniform(_impl->id, uniform_index, buffer_size, &length, &size, &type, name);

        spdlog::debug(" - Uniform {} - {}", name, uniform_index);
    }
}

Program::~Program()
{
    glDeleteProgram(_impl->id);

    delete _impl;
}

uint32_t Program::GetID() const
{
    return _impl->id;
}

void Program::Bind()
{
    glUseProgram(_impl->id);
}

void Program::Unbind()
{
    glUseProgram(0);
}

void Program::PushUniformSamplerUnit(const uint32_t unit, const uint32_t texture)
{
    glBindTextureUnit(unit, texture);
}

void Program::PushUniformS32(const std::string& name, const int32_t value)
{
    glProgramUniform1i(_impl->id, GetUniformLocation(name), value);
}

void Program::PushUniform1F32(const std::string& name, const float value)
{
    glProgramUniform1f(_impl->id, GetUniformLocation(name), value);
}

void Program::PushUniformU32(const std::string& name, const uint32_t value)
{
    glProgramUniform1ui(_impl->id, GetUniformLocation(name), value);
}

void Program::PushUniform2F32(const std::string& name, const glm::vec2& value)
{
    glProgramUniform2fv(_impl->id, GetUniformLocation(name), 1, glm::value_ptr(value));
}

void Program::PushUniform3F32(const std::string& name, const glm::vec3& value)
{
    glProgramUniform3fv(_impl->id, GetUniformLocation(name), 1, glm::value_ptr(value));
}

void Program::PushUniform4F32(const std::string& name, const glm::vec4& value)
{
    glProgramUniform4fv(_impl->id, GetUniformLocation(name), 1, glm::value_ptr(value));
}

void Program::PushUniform16F32(const std::string& name, const glm::mat4& value)
{
    glProgramUniformMatrix4fv(_impl->id, GetUniformLocation(name), 1, GL_FALSE, glm::value_ptr(value));
}

int32_t Program::GetUniformLocation(const std::string& name)
{
    auto& cache = _impl->uniform_location_cache;

    if (!cache.contains(name))
    {
        const int32_t location = glGetUniformLocation(_impl->id, name.c_str());

        if (location == kInvalidLocation)
        {
            spdlog::error("Program {} does not have uniform {}!", _impl->id, name);
            return kInvalidLocation;
        }

        cache[name] = location;

        return location;
    }

    return cache[name];
}
