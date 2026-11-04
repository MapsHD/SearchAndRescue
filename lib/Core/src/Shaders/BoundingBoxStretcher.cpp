#include <Core/Shaders.h>

static constexpr const char* const kBoundingBoxStretcherVert = R"(
#version 410 core

layout(location = 0) in vec3 in_Position;

uniform mat4 u_MVP;
uniform vec3 u_Color;
uniform mat4 u_Pose;

out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = u_Color;
    gl_Position = u_MVP * u_Pose * vec4(in_Position, 1.0);
}
)";

static constexpr const char* const kBoundingBoxStretcherFrag = R"(
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

ProgramShaderSources GetProgramShaderSources_BoundingBoxStretcher()
{
    return ProgramShaderSources{
        .vertex_source        = kBoundingBoxStretcherVert,
        .vertex_source_size   = (int32_t)strlen(kBoundingBoxStretcherVert),
        .fragment_source      = kBoundingBoxStretcherFrag,
        .fragment_source_size = (int32_t)strlen(kBoundingBoxStretcherFrag)};
}
