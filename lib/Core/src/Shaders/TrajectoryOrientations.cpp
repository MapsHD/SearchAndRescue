#include <Core/Shaders.h>

// ---------------------------------------------------------------------------
// GLSL 410 shaders
// ---------------------------------------------------------------------------
// The orientation matrix is fed as three separate vec3 vertex attributes
// (locations 1, 2, 3 = columns X, Y, Z of the rotation matrix).
// The vertex shader passes them through an interface block to the geometry
// shader, which uses them to emit the three axis line segments.
// ---------------------------------------------------------------------------

#if HDMAPPING_SEARCH_AND_RESCUE_USE_GLSL_410

static constexpr const char* const kTrajectoryOrientationsVert = R"(
#version 410 core
layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_AxisX;
layout(location = 2) in vec3 in_AxisY;
layout(location = 3) in vec3 in_AxisZ;

out BLOCK
{
    vec3 axis_x;
    vec3 axis_y;
    vec3 axis_z;
} vs_out;

void main()
{
    vs_out.axis_x = in_AxisX;
    vs_out.axis_y = in_AxisY;
    vs_out.axis_z = in_AxisZ;
    gl_Position = vec4(in_Position, 1.0);
}
)";

static constexpr const char* const kTrajectoryOrientationsGeom = R"(
#version 410 core
layout(points) in;
layout(line_strip, max_vertices = 6) out;

in BLOCK
{
    vec3 axis_x;
    vec3 axis_y;
    vec3 axis_z;
} gs_in[];

uniform mat4  u_MVP;
uniform float u_AxisLength;

out vec3 axis_Color;

void emit_axis(vec3 position, vec3 direction, vec3 color)
{
    axis_Color  = color;
    gl_Position = u_MVP * vec4(position, 1.0);
    EmitVertex();
    gl_Position = u_MVP * vec4(position + direction * u_AxisLength, 1.0);
    EmitVertex();
    EndPrimitive();
}

void main()
{
    vec3 position = gl_in[0].gl_Position.xyz;
    emit_axis(position, gs_in[0].axis_x, vec3(1.0, 0.0, 0.0));
    emit_axis(position, gs_in[0].axis_y, vec3(0.0, 1.0, 0.0));
    emit_axis(position, gs_in[0].axis_z, vec3(0.0, 0.0, 1.0));
}
)";

static constexpr const char* const kTrajectoryOrientationsFrag = R"(
#version 410 core
in  vec3 axis_Color;
layout(location = 0) out vec3 out_Color;
void main()
{
    out_Color = axis_Color;
}
)";

// ---------------------------------------------------------------------------
// GLSL 460 shaders (SSBO-free, same vertex-attribute approach)
// ---------------------------------------------------------------------------
#else

static constexpr const char* const kTrajectoryOrientationsVert = R"(
#version 460 core
layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_AxisX;
layout(location = 2) in vec3 in_AxisY;
layout(location = 3) in vec3 in_AxisZ;

layout(location = 0) out BLOCK
{
    vec3 axis_x;
    vec3 axis_y;
    vec3 axis_z;
} vs_out;

void main()
{
    vs_out.axis_x = in_AxisX;
    vs_out.axis_y = in_AxisY;
    vs_out.axis_z = in_AxisZ;
    gl_Position = vec4(in_Position, 1.0);
}
)";

static constexpr const char* const kTrajectoryOrientationsGeom = R"(
#version 460 core
layout(points) in;
layout(line_strip, max_vertices = 6) out;

layout(location = 0) in BLOCK
{
    vec3 axis_x;
    vec3 axis_y;
    vec3 axis_z;
} gs_in[];

layout(location = 0) uniform mat4  u_MVP;
layout(location = 1) uniform float u_AxisLength;

layout(location = 0) out vec3 axis_Color;

void emit_axis(vec3 position, vec3 direction, vec3 color)
{
    axis_Color  = color;
    gl_Position = u_MVP * vec4(position, 1.0);
    EmitVertex();
    gl_Position = u_MVP * vec4(position + direction * u_AxisLength, 1.0);
    EmitVertex();
    EndPrimitive();
}

void main()
{
    vec3 position = gl_in[0].gl_Position.xyz;
    emit_axis(position, gs_in[0].axis_x, vec3(1.0, 0.0, 0.0));
    emit_axis(position, gs_in[0].axis_y, vec3(0.0, 1.0, 0.0));
    emit_axis(position, gs_in[0].axis_z, vec3(0.0, 0.0, 1.0));
}
)";

static constexpr const char* const kTrajectoryOrientationsFrag = R"(
#version 460 core
layout(location = 0) in  vec3 axis_Color;
layout(location = 0) out vec3 out_Color;
void main()
{
    out_Color = axis_Color;
}
)";

#endif // HDMAPPING_SEARCH_AND_RESCUE_USE_GLSL_410

// ---------------------------------------------------------------------------
// Source accessor (unconditional -- the feature now works on both 410 and 460)
// ---------------------------------------------------------------------------
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