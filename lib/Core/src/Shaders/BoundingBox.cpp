#include <Core/Shaders.h>

static constexpr const char* const kBoundingBoxVert = R"(
#version 410 core

layout(location = 0) in vec3 in_Position;

uniform mat4 u_MVP;
uniform vec3 u_Color;

out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = u_Color;
    gl_Position = u_MVP * vec4(in_Position, 1.0);
}
)";

static constexpr const char* const kBoundingBoxFrag = R"(
#version 410 core

layout(location = 0) out vec3 out_Color;

in BLOCK
{
    vec3 color;
} 
shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";

ProgramShaderSources GetProgramShaderSources_BoundingBox()
{
    return ProgramShaderSources{
        .vertex_source        = kBoundingBoxVert,
        .vertex_source_size   = (int32_t)strlen(kBoundingBoxVert),
        .fragment_source      = kBoundingBoxFrag,
        .fragment_source_size = (int32_t)strlen(kBoundingBoxFrag)};
}
