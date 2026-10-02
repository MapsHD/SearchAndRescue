#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

struct ShaderDescriptor
{
    uint32_t    shader_type = 0;
    int32_t     source_size = 0;
    const char* source      = nullptr;
};

class Program
{
private:
    struct ProgramIMPL;

private:
    ProgramIMPL* _impl = nullptr;

public:
    explicit Program(const std::vector<ShaderDescriptor>& program_descriptor);
    ~Program();

    uint32_t GetID() const;

    void Bind();
    void Unbind();

    void PushUniformSamplerUnit(const uint32_t unit, const uint32_t texture);

    void PushUniformS32(const std::string& name, const int32_t value);
    void PushUniformU32(const std::string& name, const uint32_t value);
    void PushUniform1F32(const std::string& name, const float value);
    void PushUniform2F32(const std::string& name, const glm::vec2& value);
    void PushUniform3F32(const std::string& name, const glm::vec3& value);
    void PushUniform4F32(const std::string& name, const glm::vec4& value);
    void PushUniform16F32(const std::string& name, const glm::mat4& value);

    int32_t GetUniformLocation(const std::string& name);
};
