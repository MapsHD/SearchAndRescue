#pragma once

#include <Core/Enums.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <filesystem>

struct UserSettings
{
    struct
    {
        glm::vec3 clear_color = {0.5f, 0.5f, 0.5f};
    } opengl;

    struct
    {
        float   map_load_extent                     = 1.0f;
        int32_t map_load_decimation_factor          = 2;
        int32_t map_load_decimation_levels          = 8;
        int32_t map_load_minimum_first_level_points = 250;
        bool    map_load_use_center_extent          = true;
        int32_t trajectory_load_every_nth           = 1;
    } io;

    struct
    {
        bool  draw_enable = true;
        float scale       = 1.0f;
        float width       = 1.0f;
    } origin;

    struct
    {
        bool      draw_enable = true;
        float     scale       = 1.0f;
        float     width       = 1.0f;
        glm::vec3 color       = {1.0f, 1.0f, 1.0f};
    } target;

    struct
    {
        bool      draw_enable             = true;
        bool      draw_orientations       = false;
        float     orientation_axis_length = 0.05f;
        float     width                   = 1.0f;
        float     point_size              = 2.0f;
        glm::vec3 color                   = {1.0f, 1.0f, 1.0f};

        TrajectoryDisplayMode display_mode = TrajectoryDisplayMode::TRAJECTORY_DISPLAY_MODE_LINE_STRIP;

        // helper for JSON loading; never used directly
        int32_t display_mode_value = 0;
    } trajectory;

    struct
    {
        bool draw_enable = true;
    } measurements;

    struct
    {
        bool      draw_enable      = true;
        bool      draw_enable_bbox = true;
        float     bbox_width       = 1.0f;
        glm::vec3 bbox_color       = {0.0f, 1.0f, 1.0f};
    } stretcher;

    struct
    {
        float point_size = 1.0f;

        float bbox_width                  = 1.0f;
        float bbox_width_in_obb           = 4.0f;
        float bbox_width_in_obb_proximity = 3.0f;

        bool draw_enable_bbox                  = false;
        bool draw_enable_bbox_out              = false;
        bool draw_enable_bbox_in_obb           = false;
        bool draw_enable_bbox_in_obb_proximity = false;

        bool draw_enable_pc                  = true;
        bool draw_enable_pc_out              = true;
        bool draw_enable_pc_in_obb           = true;
        bool draw_enable_pc_in_obb_proximity = true;

        PointCloudDisplayMode display_mode = PointCloudDisplayMode::POINT_CLOUD_DISPLAY_MODE_COLOR_MAP;

        ColorMapType colormap = ColorMapType::COLOR_MAP_TYPE_TURBO;

        // helper for JSON loading; never used directly
        int32_t display_mode_value = 0;
        int32_t colormap_value     = 0;
    } point_cloud;

    struct
    {
        float radious = 3.0f;

        float     points_size  = 8.0f;
        glm::vec3 points_color = {1.0f, 0.0f, 1.0f};
    } collision;
};

void UserSettingsImGUI(UserSettings& user_settings, bool& open);

bool UserSettingsSaveJSON(const std::filesystem::path& path, const UserSettings& user_settings);
bool UserSettingsLoadJSON(const std::filesystem::path& path, UserSettings& user_settings_out);
