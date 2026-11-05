#include <Core/Shaders.h>

static constexpr const char* const kTrajectoryVert = R"(
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

static constexpr const char* const kTrajectoryFrag = R"(
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

ProgramShaderSources GetProgramShaderSources_Trajectory()
{
    return ProgramShaderSources{
        .vertex_source        = kTrajectoryVert,
        .vertex_source_size   = (int32_t)strlen(kTrajectoryVert),
        .fragment_source      = kTrajectoryFrag,
        .fragment_source_size = (int32_t)strlen(kTrajectoryFrag)};
}
