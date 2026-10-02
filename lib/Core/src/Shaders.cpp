// clang-format off
#include <glad/glad.h>
// clang-format on

#include <Core/Shaders.h>

#include <Core/OpenGL/Program.h>

#include <cstring>
#include <vector>

Program* make_program(const ProgramShaderSources& sources)
{
    std::vector<ShaderDescriptor> shaders = {};

    shaders.push_back({GL_VERTEX_SHADER, sources.vertex_source_size, sources.vertex_source});
    shaders.push_back({GL_FRAGMENT_SHADER, sources.fragment_source_size, sources.fragment_source});

    if (sources.geometry_source)
    {
        shaders.push_back({GL_GEOMETRY_SHADER, sources.geometry_source_size, sources.geometry_source});
    }

    return new Program(shaders);
}
