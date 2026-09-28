#include <cave-traversal-tool/PFDWrapper.h>
#include <cave-traversal-tool/UserSettings.h>

#include <imgui.h>

#include <glm/gtc/type_ptr.hpp>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <limits>

namespace
{
    using json = nlohmann::json;

    json Vec3ToJSON(const glm::vec3& value)
    {
        return json::array({value.x, value.y, value.z});
    }

    const json* FindObject(const json& parent, const char* key)
    {
        if (!parent.is_object())
            return nullptr;
        const auto it = parent.find(key);
        return it != parent.end() && it->is_object() ? &*it : nullptr;
    }

    bool ReadSetting(const json& parent, const char* key, bool& out)
    {
        if (!parent.is_object())
            return false;
        const auto it = parent.find(key);
        if (it == parent.end() || !it->is_boolean())
            return false;
        out = it->get<bool>();
        return true;
    }

    bool ReadSetting(const json& parent, const char* key, float& out)
    {
        if (!parent.is_object())
            return false;
        const auto it = parent.find(key);
        if (it == parent.end() || !it->is_number())
            return false;
        out = it->get<float>();
        return true;
    }

    bool ReadSetting(const json& parent, const char* key, int32_t& out)
    {
        if (!parent.is_object())
            return false;
        const auto it = parent.find(key);
        if (it == parent.end() || !it->is_number_integer())
            return false;
        if (it->is_number_unsigned())
        {
            if (it->get<uint64_t>() > static_cast<uint64_t>(std::numeric_limits<int32_t>::max()))
                return false;
        }
        else
        {
            const auto value = it->get<int64_t>();
            if (value < std::numeric_limits<int32_t>::min() || value > std::numeric_limits<int32_t>::max())
                return false;
        }
        out = it->get<int32_t>();
        return true;
    }

    void Vec3FromJSON(const json* parent, const char* key, glm::vec3& out)
    {
        out = glm::vec3(1.0f);
        if (!parent)
            return;
        const auto it = parent->find(key);
        if (it == parent->end() || !it->is_array() || it->size() != 3 ||
            !(*it)[0].is_number() || !(*it)[1].is_number() || !(*it)[2].is_number())
            return;
        out = glm::vec3((*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>());
    }
}

void UserSettingsImGUI(UserSettings& user_settings, bool& open)
{
    if (ImGui::Begin("User settings", &open))
    {
        if (ImGui::TreeNode("Save / Load"))
        {
            if (ImGui::Button("Save to JSON"))
            {
                std::string selected;
                if (PFDSaveFile("Save user settings", "user-settings.json", "JSON files", "*.json", selected))
                {
                    std::filesystem::path path(selected);
                    if (!path.has_extension())
                        path += ".json";
                    if (!UserSettingsSaveJSON(path, user_settings))
                        spdlog::warn("Failed to save user settings to {}", path.string());
                }
            }

            if (ImGui::Button("Load from JSON"))
            {
                std::string selected;
                if (PFDOpenFile("Load user settings", "JSON files", "*.json", selected) && !UserSettingsLoadJSON(selected, user_settings))
                    spdlog::warn("Failed to load user settings from {}", selected);
            }
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("OpenGL"))
        {
            ImGui::ColorEdit3("Clear color", glm::value_ptr(user_settings.opengl.clear_color));
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("IO"))
        {
            ImGui::DragFloat("Map load extent", &user_settings.io.map_load_extent, 0.01f, 0.1f, FLT_MAX, "%.3f");
            ImGui::DragInt("Map load decimation factor", &user_settings.io.map_load_decimation_factor, 1.0f, 1, INT32_MAX);
            ImGui::DragInt("Map load decimation levels", &user_settings.io.map_load_decimation_levels, 1.0f, 1, INT32_MAX);
            ImGui::DragInt("Map load minimum first level points", &user_settings.io.map_load_minimum_first_level_points, 1.0f, 1, INT32_MAX);
            ImGui::Checkbox("Map load use center extent", &user_settings.io.map_load_use_center_extent);
            ImGui::DragInt("Trajectory load every N-th", &user_settings.io.trajectory_load_every_nth, 1.0f, 1, INT32_MAX);
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Origin"))
        {
            ImGui::Checkbox("Enable draw", &user_settings.origin.draw_enable);
            ImGui::BeginDisabled(!user_settings.origin.draw_enable);
            {
                ImGui::DragFloat("Scale", &user_settings.origin.scale, 0.1f, 1.0f, FLT_MAX);
                ImGui::DragFloat("Width", &user_settings.origin.width, 0.25f, 1.0f, 8.0f);
            }
            ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Target"))
        {
            ImGui::Checkbox("Enable draw", &user_settings.target.draw_enable);
            ImGui::BeginDisabled(!user_settings.target.draw_enable);
            {
                ImGui::DragFloat("Scale", &user_settings.target.scale, 0.1f, 1.0f, FLT_MAX);
                ImGui::DragFloat("Width", &user_settings.target.width, 0.25f, 1.0f, 8.0f);
                ImGui::ColorEdit3("Color", glm::value_ptr(user_settings.target.color));
            }
            ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Trajectory"))
        {
            ImGui::Checkbox("Enable draw", &user_settings.trajectory.draw_enable);
            ImGui::BeginDisabled(!user_settings.trajectory.draw_enable);
            {
                ImGui::DragFloat("Width", &user_settings.trajectory.width, 0.25f, 1.0f, 8.0f);
                ImGui::ColorEdit3("Color", glm::value_ptr(user_settings.trajectory.color));
            }
            ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Stretcher"))
        {
            ImGui::Checkbox("Enable draw", &user_settings.stretcher.draw_enable);
            ImGui::Checkbox("Enable draw (BBOX)", &user_settings.stretcher.draw_enable_bbox);
            ImGui::BeginDisabled(!user_settings.stretcher.draw_enable_bbox);
            {
                ImGui::DragFloat("Width", &user_settings.stretcher.bbox_width, 0.25f, 1.0f, 8.0f);
                ImGui::ColorEdit3("Color", glm::value_ptr(user_settings.stretcher.bbox_color));
            }
            ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Point cloud"))
        {
            ImGui::DragFloat("Point size", &user_settings.point_cloud.point_size, 0.25f, 1.0f, 4.0f);

            {
                const char* const display_modes = "Intensity\0Color map\0Color map * intensity\0Color map (position)\0Color map (position) * intensity\0";
                int32_t           display_mode  = static_cast<int32_t>(user_settings.point_cloud.display_mode);
                if (ImGui::Combo("Display mode", &display_mode, display_modes))
                {
                    user_settings.point_cloud.display_mode = static_cast<PointCloudDisplayMode>(display_mode);
                }
            }

            ImGui::DragFloat("BBOX width", &user_settings.point_cloud.bbox_width, 0.25f, 1.0f, 8.0f);
            ImGui::DragFloat("BBOX width in OBB", &user_settings.point_cloud.bbox_width_in_obb, 0.25f, 1.0f, 8.0f);
            ImGui::DragFloat("BBOX width in OBB proximity", &user_settings.point_cloud.bbox_width_in_obb_proximity, 0.25f, 1.0f, 8.0f);

            ImGui::Checkbox("Enable draw BBOX (master)", &user_settings.point_cloud.draw_enable_bbox);
            ImGui::BeginDisabled(!user_settings.point_cloud.draw_enable_bbox);
            {
                ImGui::Checkbox("Enable draw BBOX (outside)", &user_settings.point_cloud.draw_enable_bbox_out);
                ImGui::Checkbox("Enable draw BBOX (in OBB)", &user_settings.point_cloud.draw_enable_bbox_in_obb);
                ImGui::Checkbox("Enable draw BBOX (in OBB proximity)", &user_settings.point_cloud.draw_enable_bbox_in_obb_proximity);
            }
            ImGui::EndDisabled();

            ImGui::Checkbox("Enable draw PC (master)", &user_settings.point_cloud.draw_enable_pc);
            ImGui::BeginDisabled(!user_settings.point_cloud.draw_enable_pc);
            {
                ImGui::Checkbox("Enable draw PC (outside)", &user_settings.point_cloud.draw_enable_pc_out);
                ImGui::Checkbox("Enable draw PC (in OBB)", &user_settings.point_cloud.draw_enable_pc_in_obb);
                ImGui::Checkbox("Enable draw PC (in OBB proximity)", &user_settings.point_cloud.draw_enable_pc_in_obb_proximity);
            }
            ImGui::EndDisabled();

            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Collision"))
        {
            ImGui::DragFloat("Radious", &user_settings.collision.radious, 0.1f, 0.0f, FLT_MAX);
            ImGui::DragFloat("Points size", &user_settings.collision.points_size, 0.1f, 0.1f, 16.0f, "%.1f");
            ImGui::ColorEdit3("Points color", glm::value_ptr(user_settings.collision.points_color));
            ImGui::TreePop();
        }
    }
    ImGui::End();
}

bool UserSettingsSaveJSON(const std::filesystem::path& path, const UserSettings& user_settings)
{
    const nlohmann::json settings = {
        {"opengl",
         {{"clear_color", Vec3ToJSON(user_settings.opengl.clear_color)}}},
        {"io",
         {{"map_load_extent", user_settings.io.map_load_extent},
          {"map_load_decimation_factor", user_settings.io.map_load_decimation_factor},
          {"map_load_decimation_levels", user_settings.io.map_load_decimation_levels},
          {"map_load_minimum_first_level_points", user_settings.io.map_load_minimum_first_level_points},
          {"map_load_use_center_extent", user_settings.io.map_load_use_center_extent},
          {"trajectory_load_every_nth", user_settings.io.trajectory_load_every_nth}}},
        {"origin",
         {{"draw_enable", user_settings.origin.draw_enable},
          {"scale", user_settings.origin.scale},
          {"width", user_settings.origin.width}}},
        {"target",
         {{"draw_enable", user_settings.target.draw_enable},
          {"scale", user_settings.target.scale},
          {"width", user_settings.target.width},
          {"color", Vec3ToJSON(user_settings.target.color)}}},
        {"trajectory",
         {{"draw_enable", user_settings.trajectory.draw_enable},
          {"width", user_settings.trajectory.width},
          {"color", Vec3ToJSON(user_settings.trajectory.color)}}},
        {"stretcher",
         {{"draw_enable", user_settings.stretcher.draw_enable},
          {"draw_enable_bbox", user_settings.stretcher.draw_enable_bbox},
          {"bbox_width", user_settings.stretcher.bbox_width},
          {"bbox_color", Vec3ToJSON(user_settings.stretcher.bbox_color)}}},
        {"point_cloud",
         {{"point_size", user_settings.point_cloud.point_size},
          {"bbox_width", user_settings.point_cloud.bbox_width},
          {"bbox_width_in_obb", user_settings.point_cloud.bbox_width_in_obb},
          {"bbox_width_in_obb_proximity", user_settings.point_cloud.bbox_width_in_obb_proximity},
          {"draw_enable_bbox", user_settings.point_cloud.draw_enable_bbox},
          {"draw_enable_bbox_out", user_settings.point_cloud.draw_enable_bbox_out},
          {"draw_enable_bbox_in_obb", user_settings.point_cloud.draw_enable_bbox_in_obb},
          {"draw_enable_bbox_in_obb_proximity", user_settings.point_cloud.draw_enable_bbox_in_obb_proximity},
          {"draw_enable_pc", user_settings.point_cloud.draw_enable_pc},
          {"draw_enable_pc_out", user_settings.point_cloud.draw_enable_pc_out},
          {"draw_enable_pc_in_obb", user_settings.point_cloud.draw_enable_pc_in_obb},
          {"draw_enable_pc_in_obb_proximity", user_settings.point_cloud.draw_enable_pc_in_obb_proximity},
          {"display_mode", static_cast<int32_t>(user_settings.point_cloud.display_mode)}}},
        {"collision",
         {{"radious", user_settings.collision.radious},
          {"points_size", user_settings.collision.points_size},
          {"points_color", Vec3ToJSON(user_settings.collision.points_color)}}}};

    std::ofstream file(path);
    if (!file)
        return false;
    file << settings.dump(4) << '\n';
    file.close();
    return file.good();
}

bool UserSettingsLoadJSON(const std::filesystem::path& path, UserSettings& user_settings_out)
{
    std::ifstream file(path);
    if (!file)
        return false;

    const json settings = json::parse(file, nullptr, false);
    if (settings.is_discarded() || file.bad())
        return false;

    const json* opengl      = FindObject(settings, "opengl");
    const json* io          = FindObject(settings, "io");
    const json* origin      = FindObject(settings, "origin");
    const json* target      = FindObject(settings, "target");
    const json* trajectory  = FindObject(settings, "trajectory");
    const json* stretcher   = FindObject(settings, "stretcher");
    const json* point_cloud = FindObject(settings, "point_cloud");
    const json* collision   = FindObject(settings, "collision");
    if (!io || !origin || !target || !trajectory || !stretcher || !point_cloud || !collision)
        return false;

    UserSettings loaded;
    if (!(ReadSetting(*io, "map_load_extent", loaded.io.map_load_extent) &&
          ReadSetting(*io, "map_load_decimation_factor", loaded.io.map_load_decimation_factor) &&
          ReadSetting(*io, "map_load_decimation_levels", loaded.io.map_load_decimation_levels) &&
          ReadSetting(*io, "map_load_minimum_first_level_points", loaded.io.map_load_minimum_first_level_points) &&
          ReadSetting(*io, "map_load_use_center_extent", loaded.io.map_load_use_center_extent) &&
          ReadSetting(*io, "trajectory_load_every_nth", loaded.io.trajectory_load_every_nth) &&
          ReadSetting(*origin, "draw_enable", loaded.origin.draw_enable) &&
          ReadSetting(*origin, "scale", loaded.origin.scale) &&
          ReadSetting(*origin, "width", loaded.origin.width) &&
          ReadSetting(*target, "draw_enable", loaded.target.draw_enable) &&
          ReadSetting(*target, "scale", loaded.target.scale) &&
          ReadSetting(*target, "width", loaded.target.width) &&
          ReadSetting(*trajectory, "draw_enable", loaded.trajectory.draw_enable) &&
          ReadSetting(*trajectory, "width", loaded.trajectory.width) &&
          ReadSetting(*stretcher, "draw_enable", loaded.stretcher.draw_enable) &&
          ReadSetting(*stretcher, "draw_enable_bbox", loaded.stretcher.draw_enable_bbox) &&
          ReadSetting(*stretcher, "bbox_width", loaded.stretcher.bbox_width) &&
          ReadSetting(*point_cloud, "point_size", loaded.point_cloud.point_size) &&
          ReadSetting(*point_cloud, "bbox_width", loaded.point_cloud.bbox_width) &&
          ReadSetting(*point_cloud, "bbox_width_in_obb", loaded.point_cloud.bbox_width_in_obb) &&
          ReadSetting(*point_cloud, "bbox_width_in_obb_proximity", loaded.point_cloud.bbox_width_in_obb_proximity) &&
          ReadSetting(*point_cloud, "draw_enable_bbox", loaded.point_cloud.draw_enable_bbox) &&
          ReadSetting(*point_cloud, "draw_enable_bbox_out", loaded.point_cloud.draw_enable_bbox_out) &&
          ReadSetting(*point_cloud, "draw_enable_bbox_in_obb", loaded.point_cloud.draw_enable_bbox_in_obb) &&
          ReadSetting(*point_cloud, "draw_enable_bbox_in_obb_proximity", loaded.point_cloud.draw_enable_bbox_in_obb_proximity) &&
          ReadSetting(*point_cloud, "draw_enable_pc", loaded.point_cloud.draw_enable_pc) &&
          ReadSetting(*point_cloud, "draw_enable_pc_out", loaded.point_cloud.draw_enable_pc_out) &&
          ReadSetting(*point_cloud, "draw_enable_pc_in_obb", loaded.point_cloud.draw_enable_pc_in_obb) &&
          ReadSetting(*point_cloud, "draw_enable_pc_in_obb_proximity", loaded.point_cloud.draw_enable_pc_in_obb_proximity) &&
          ReadSetting(*point_cloud, "display_mode", loaded.point_cloud.display_mode_value) &&
          ReadSetting(*collision, "radious", loaded.collision.radious) &&
          ReadSetting(*collision, "points_size", loaded.collision.points_size)))
        return false;

    Vec3FromJSON(opengl, "clear_color", loaded.opengl.clear_color);
    Vec3FromJSON(target, "color", loaded.target.color);
    Vec3FromJSON(trajectory, "color", loaded.trajectory.color);
    Vec3FromJSON(stretcher, "bbox_color", loaded.stretcher.bbox_color);
    Vec3FromJSON(collision, "points_color", loaded.collision.points_color);

    if (loaded.point_cloud.display_mode_value < 0 || loaded.point_cloud.display_mode_value > 4)
        return false;
    loaded.point_cloud.display_mode = static_cast<PointCloudDisplayMode>(loaded.point_cloud.display_mode_value);

    user_settings_out = loaded;
    return true;
}
