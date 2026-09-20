#include <cave-traversal-tool/Shaders.h>

#include <cstring>

static constexpr const char* const kBoundingBoxStretcherVert = R"(
#version 460 core

layout(location = 0) in vec3 in_Position;

layout(location = 0) uniform mat4 u_MVP   = mat4(1.0f);
layout(location = 1) uniform vec3 u_Color = vec3(1.0f);
layout(location = 2) uniform mat4 u_Pose  = mat4(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = u_Color;

    gl_Position = u_MVP * u_Pose * vec4(in_Position, 1.0f);
}
)";

static constexpr const char* const kBoundingBoxStretcherFrag = R"(
#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";

static constexpr const char* const kBoundingBoxVert = R"(
#version 460 core

layout(location = 0) in vec3 in_Position;

layout(location = 0) uniform mat4 u_MVP   = mat4(1.0f);
layout(location = 1) uniform vec3 u_Color = vec3(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = u_Color;

    gl_Position = u_MVP * vec4(in_Position, 1.0f);
}
)";

static constexpr const char* const kBoundingBoxFrag = R"(
#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";

static constexpr const char* const kCameraTargetVert = R"(
#version 460 core

layout(location = 0) in vec3 in_Position;

layout(location = 0) uniform mat4 u_MVP         = mat4(1.0f);
layout(location = 1) uniform vec3 u_Translation = vec3(0.0f);
layout(location = 2) uniform float u_Scale      = float(1.0f);
layout(location = 3) uniform vec3 u_Color       = vec3(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = u_Color;

    gl_Position = u_MVP * vec4(u_Scale * in_Position + u_Translation, 1.0f);
}
)";

static constexpr const char* const kCameraTargetFrag = R"(
#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";

static constexpr const char* const kOriginVert = R"(
#version 460 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Color;

layout(location = 0) uniform mat4 u_MVP    = mat4(1.0f);
layout(location = 1) uniform float u_Scale = float(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = in_Color;

    gl_Position = u_MVP * vec4(u_Scale * in_Position, 1.0f);
}
)";

static constexpr const char* const kOriginFrag = R"(
#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";

static constexpr const char* const kPointCloudVert = R"(
#version 460 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in float in_Intensity;

layout(location = 0) uniform mat4 u_MVP = mat4(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    shared_data.color = vec3(in_Intensity);

    gl_Position = u_MVP * vec4(in_Position, 1.0f);
}
)";

static constexpr const char* const kPointCloudFrag = R"(
#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
} shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";

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
} shared_data;

void main()
{
    out_Color = shared_data.color;
}
)";

static constexpr const char* const kTrajectoryVert = R"(
#version 460 core

layout(location = 0) in vec3 in_Position;

layout(location = 0) uniform mat4 u_MVP   = mat4(1.0f);
layout(location = 1) uniform vec3 u_Color = vec3(1.0f);

layout(location = 0) out BLOCK
{
    vec3 color;
}
shared_data;

void main()
{
    shared_data.color = u_Color;

    gl_Position = u_MVP * vec4(in_Position, 1.0f);
}
)";

static constexpr const char* const kTrajectoryFrag = R"(
#version 460 core

layout(location = 0) out vec3 out_Color;

layout(location = 0) in BLOCK
{
    vec3 color;
} shared_data;

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

ProgramShaderSources GetProgramShaderSources_BoundingBox()
{
    return ProgramShaderSources{
        .vertex_source        = kBoundingBoxVert,
        .vertex_source_size   = (int32_t)strlen(kBoundingBoxVert),
        .fragment_source      = kBoundingBoxFrag,
        .fragment_source_size = (int32_t)strlen(kBoundingBoxFrag)};
}

ProgramShaderSources GetProgramShaderSources_CameraTarger()
{
    return ProgramShaderSources{
        .vertex_source        = kCameraTargetVert,
        .vertex_source_size   = (int32_t)strlen(kCameraTargetVert),
        .fragment_source      = kCameraTargetFrag,
        .fragment_source_size = (int32_t)strlen(kCameraTargetFrag)};
}

ProgramShaderSources GetProgramShaderSources_Origin()
{
    return ProgramShaderSources{
        .vertex_source        = kOriginVert,
        .vertex_source_size   = (int32_t)strlen(kOriginVert),
        .fragment_source      = kOriginFrag,
        .fragment_source_size = (int32_t)strlen(kOriginFrag)};
}

ProgramShaderSources GetProgramShaderSources_PointCloud()
{
    return ProgramShaderSources{
        .vertex_source        = kPointCloudVert,
        .vertex_source_size   = (int32_t)strlen(kPointCloudVert),
        .fragment_source      = kPointCloudFrag,
        .fragment_source_size = (int32_t)strlen(kPointCloudFrag)};
}

ProgramShaderSources GetProgramShaderSources_Stretcher()
{
    return ProgramShaderSources{
        .vertex_source        = kStretcherVert,
        .vertex_source_size   = (int32_t)strlen(kStretcherVert),
        .fragment_source      = kStretcherFrag,
        .fragment_source_size = (int32_t)strlen(kStretcherFrag)};
}

ProgramShaderSources GetProgramShaderSources_Trajectory()
{
    return ProgramShaderSources{
        .vertex_source        = kTrajectoryVert,
        .vertex_source_size   = (int32_t)strlen(kTrajectoryVert),
        .fragment_source      = kTrajectoryFrag,
        .fragment_source_size = (int32_t)strlen(kTrajectoryFrag)};
}
