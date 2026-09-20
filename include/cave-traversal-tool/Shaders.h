#pragma once

#include <cstdint>

struct ProgramShaderSources
{
    const char* vertex_source      = nullptr;
    int32_t     vertex_source_size = 0;

    const char* fragment_source      = nullptr;
    int32_t     fragment_source_size = 0;
};

ProgramShaderSources GetProgramShaderSources_BoundingBoxStretcher();
ProgramShaderSources GetProgramShaderSources_BoundingBox();
ProgramShaderSources GetProgramShaderSources_CameraTarger();
ProgramShaderSources GetProgramShaderSources_Origin();
ProgramShaderSources GetProgramShaderSources_PointCloud();
ProgramShaderSources GetProgramShaderSources_Stretcher();
ProgramShaderSources GetProgramShaderSources_Trajectory();
