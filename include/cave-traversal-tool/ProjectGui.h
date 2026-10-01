#pragma once

#include <cave-traversal-tool/Project.h>

// ImGui panel for the project data : viewport layout / camera modes / plane controls,
// file input / output, level of detail and trajectory playback controls
void ProjectDataImGUI(ProjectData& project_data, const UserSettings& user_settings, bool& open);
