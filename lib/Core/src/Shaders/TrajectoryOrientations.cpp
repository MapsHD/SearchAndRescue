#include <Core/Shaders.h>

static constexpr const char* const kTrajectoryOrientationsVert = R"(
#version 460 core
layout(location = 0) in vec3 in_Position;

void main()
{
    gl_Position = vec4(in_Position, 1.0);
}
)";

static constexpr const char* const kTrajectoryOrientationsGeom = R"(
#version 460 core
layout(points) in;
layout(line_strip, max_vertices = 6) out;

layout(std430, binding = 0) readonly buffer Orientations
{
    mat3x4 orientation[];
};

uniform mat4 u_MVP;
uniform float u_AxisLength;
layout(location = 0) out vec3 axis_Color;

void emit_axis(vec3 position, vec3 direction, vec3 color)
{
    axis_Color = color;
    gl_Position = u_MVP * vec4(position, 1.0);
    EmitVertex();
    gl_Position = u_MVP * vec4(position + direction * u_AxisLength, 1.0);
    EmitVertex();
    EndPrimitive();
}

void main()
{
    vec3 position = gl_in[0].gl_Position.xyz;
    mat3x4 rotation = orientation[gl_PrimitiveIDIn];
    emit_axis(position, rotation[0].xyz, vec3(1.0, 0.0, 0.0));
    emit_axis(position, rotation[1].xyz, vec3(0.0, 1.0, 0.0));
    emit_axis(position, rotation[2].xyz, vec3(0.0, 0.0, 1.0));
}
)";

static constexpr const char* const kTrajectoryOrientationsFrag = R"(
#version 460 core
layout(location = 0) in vec3 axis_Color;
layout(location = 0) out vec3 out_Color;
void main()
{
    out_Color = axis_Color;
}
)";

#if HDMAPPING_SEARCH_AND_RESCUE_USE_GLSL_410 == 0
ProgramShaderSources GetProgramShaderSources_TrajectoryOrientations()
{
    return ProgramShaderSources{
        .vertex_source        = kTrajectoryOrientationsVert,
        .vertex_source_size   = (int32_t)strlen(kTrajectoryOrientationsVert),
        .geometry_source      = kTrajectoryOrientationsGeom,
        .geometry_source_size = (int32_t)strlen(kTrajectoryOrientationsGeom),
        .fragment_source      = kTrajectoryOrientationsFrag,
        .fragment_source_size = (int32_t)strlen(kTrajectoryOrientationsFrag)};
}
#endif