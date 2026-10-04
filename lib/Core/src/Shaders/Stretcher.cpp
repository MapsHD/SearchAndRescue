#include <Core/Shaders.h>

#if HDMAPPING_SEARCH_AND_RESCUE_USE_GLSL_410
static constexpr const char* const kStretcherVert = R"(
#version 410 core

layout(location = 0) in vec3 in_Position;

layout(location = 1) in vec3 in_Color;

uniform mat4 u_MVP;

uniform mat4 u_Pose;

out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = in_Color;
    gl_Position = u_MVP * u_Pose * vec4(in_Position, 1.0);
}
)";

static constexpr const char* const kStretcherFrag = R"(
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
#else
static constexpr const char* const kStretcherVert = R"(
#version 460 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Color;

layout(location = 0) uniform mat4 u_MVP  = mat4(1.0f);
layout(location = 1) uniform mat4 u_Pose = mat4(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = in_Color;

    gl_Position = u_MVP * u_Pose * vec4(in_Position, 1.0f);
}
)";

static constexpr const char* const kStretcherFrag = R"(
#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";
#endif

ProgramShaderSources GetProgramShaderSources_Stretcher()
{
    return ProgramShaderSources{
        .vertex_source        = kStretcherVert,
        .vertex_source_size   = (int32_t)strlen(kStretcherVert),
        .fragment_source      = kStretcherFrag,
        .fragment_source_size = (int32_t)strlen(kStretcherFrag)};
}
