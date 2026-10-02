#pragma once

#include <cstdint>

#include <cave-traversal-tool/OpenGL/Program.h>

struct ProgramShaderSources
{
    const char* vertex_source      = nullptr;
    int32_t     vertex_source_size = 0;

    const char* geometry_source      = nullptr;
    int32_t     geometry_source_size = 0;

    const char* fragment_source      = nullptr;
    int32_t     fragment_source_size = 0;
};

ProgramShaderSources GetProgramShaderSources_BoundingBoxStretcher();
ProgramShaderSources GetProgramShaderSources_BoundingBox();
ProgramShaderSources GetProgramShaderSources_CameraTarger();
ProgramShaderSources GetProgramShaderSources_Origin();
ProgramShaderSources GetProgramShaderSources_PointCloud();
ProgramShaderSources GetProgramShaderSources_PointCloudColorMap();
ProgramShaderSources GetProgramShaderSources_Stretcher();
ProgramShaderSources GetProgramShaderSources_Trajectory();
ProgramShaderSources GetProgramShaderSources_TrajectoryOrientations();

// Colored lines : per-vertex colour instead of a single u_Color uniform (used by measurements)
ProgramShaderSources GetProgramShaderSources_ColoredLine();

// Helper : compile a vertex+fragment program, optionally with a geometry shader
Program* make_program(const ProgramShaderSources& sources);
