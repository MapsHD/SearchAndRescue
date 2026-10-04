#include <Core/PFDWrapper.h>
#include <Core/UserSettings.h>

#include <imgui.h>

#include <glm/gtc/type_ptr.hpp>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <limits>

namespace
{
    using json = nlohmann::json;

    constexpr float PICK_RADIUS_M_MIN  = 0.001f;
    constexpr float PICK_RADIUS_M_MAX  = 1.0f;
    constexpr float PICK_RADIUS_PX_MIN = 0.5f;
    constexpr float PICK_RADIUS_PX_MAX = 50.0f;

    json Vec3ToJSON(const glm::vec3& value)
    {
        return json::array({value.x, value.y, value.z});
    }

    json PickToleranceToJSON(const PointPickTolerance& tolerance)
    {
        return {
            {"mode", static_cast<int32_t>(tolerance.mode)},
            {"radius_m", tolerance.radius_m},
            {"radius_px", tolerance.radius_px}};
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

    void PickToleranceImGUI(const char* label, PointPickTolerance& tolerance)
    {
        if (!ImGui::TreeNode(label))
            return;

        const char* const modes = "World (m)\0Screen (px)\0Larger of both\0";
        int32_t           mode  = static_cast<int32_t>(tolerance.mode);
        if (ImGui::Combo("Tolerance mode", &mode, modes))
        {
            tolerance.mode = static_cast<PickToleranceMode>(mode);
        }
        ImGui::SetItemTooltip(
            "World : fixed radius in metres.\n"
            "Screen : radius in pixels, grows with the distance to the camera.\n"
            "Larger of both : the bigger of the two at every distance.");

        ImGui::BeginDisabled(tolerance.mode == PickToleranceMode::PICK_TOLERANCE_MODE_SCREEN);
        ImGui::SliderFloat("Radius (m)", &tolerance.radius_m, PICK_RADIUS_M_MIN, PICK_RADIUS_M_MAX, "%.3f", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
        ImGui::EndDisabled();

        ImGui::BeginDisabled(tolerance.mode == PickToleranceMode::PICK_TOLERANCE_MODE_WORLD);
        ImGui::SliderFloat("Radius (px)", &tolerance.radius_px, PICK_RADIUS_PX_MIN, PICK_RADIUS_PX_MAX, "%.1f", ImGuiSliderFlags_AlwaysClamp);
        ImGui::EndDisabled();

        ImGui::TreePop();
    }
}

void UserSettingsImGUI(UserSettings& user_settings, bool& open)
{
    if (ImGui::Begin("User settings", &open))
    {
        if (ImGui::TreeNode("Save / Load / Reset"))
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

            ImGui::SameLine();
            if (ImGui::Button("Load from JSON"))
            {
                std::string selected;
                if (PFDOpenFile("Load user settings", "JSON files", "*.json", selected) && !UserSettingsLoadJSON(selected, user_settings))
                    spdlog::warn("Failed to load user settings from {}", selected);
            }

            ImGui::SameLine();
            if (ImGui::Button("Reset to defaults"))
            {
                user_settings = UserSettings{};
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
            // ImGui::BeginDisabled(!user_settings.origin.draw_enable);
            {
                ImGui::DragFloat("Scale", &user_settings.origin.scale, 0.1f, 1.0f, FLT_MAX);
                ImGui::DragFloat("Width", &user_settings.origin.width, 0.25f, 1.0f, 8.0f);
            }
            // ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Target"))
        {
            ImGui::Checkbox("Enable draw", &user_settings.target.draw_enable);
            // ImGui::BeginDisabled(!user_settings.target.draw_enable);
            {
                ImGui::DragFloat("Scale", &user_settings.target.scale, 0.1f, 1.0f, FLT_MAX);
                ImGui::DragFloat("Width", &user_settings.target.width, 0.25f, 1.0f, 8.0f);
                ImGui::ColorEdit3("Color", glm::value_ptr(user_settings.target.color));
            }
            // ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Trajectory"))
        {
            ImGui::Checkbox("Enable draw", &user_settings.trajectory.draw_enable);
            // ImGui::BeginDisabled(!user_settings.trajectory.draw_enable);
            {
                const char* const display_modes = "Line strip\0Points\0";
                int32_t           display_mode  = static_cast<int32_t>(user_settings.trajectory.display_mode);
                if (ImGui::Combo("Display mode", &display_mode, display_modes))
                {
                    user_settings.trajectory.display_mode = static_cast<TrajectoryDisplayMode>(display_mode);
                }

                ImGui::DragFloat("Width", &user_settings.trajectory.width, 0.25f, 1.0f, 8.0f);
                ImGui::DragFloat("Point size", &user_settings.trajectory.point_size, 0.25f, 1.0f, 16.0f);
                ImGui::ColorEdit3("Color", glm::value_ptr(user_settings.trajectory.color));
                ImGui::Checkbox("Draw orientation axes", &user_settings.trajectory.draw_orientations);
                ImGui::DragFloat("Orientation axis length (m)", &user_settings.trajectory.orientation_axis_length, 0.01f, 0.001f, 100.0f, "%.3f");
            }
            // ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Measurements"))
        {
            ImGui::Checkbox("Enable display", &user_settings.measurements.draw_enable);
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Picking"))
        {
            PickToleranceImGUI("Trajectory (Alt + LMB / RMB)", user_settings.picking.trajectory);
            PickToleranceImGUI("Point snap (Ctrl + RMB)", user_settings.picking.point_snap);
            PickToleranceImGUI("Measurement (Shift + LMB)", user_settings.picking.measurement);
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Stretcher"))
        {
            ImGui::Checkbox("Enable draw", &user_settings.stretcher.draw_enable);
            ImGui::Checkbox("Enable draw (BBOX)", &user_settings.stretcher.draw_enable_bbox);
            // ImGui::BeginDisabled(!user_settings.stretcher.draw_enable_bbox);
            {
                ImGui::DragFloat("Width", &user_settings.stretcher.bbox_width, 0.25f, 1.0f, 8.0f);
                ImGui::ColorEdit3("Color", glm::value_ptr(user_settings.stretcher.bbox_color));
            }
            // ImGui::EndDisabled();
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

            {
                const char* const colormaps = "Turbo\0Viridis\0Plasma\0Magma\0Inferno\0";
                int32_t           colormap  = static_cast<int32_t>(user_settings.point_cloud.colormap);
                if (ImGui::Combo("Colormap", &colormap, colormaps))
                {
                    user_settings.point_cloud.colormap = static_cast<ColorMapType>(colormap);
                }
            }

            ImGui::DragFloat("BBOX width", &user_settings.point_cloud.bbox_width, 0.25f, 1.0f, 8.0f);
            ImGui::DragFloat("BBOX width in OBB", &user_settings.point_cloud.bbox_width_in_obb, 0.25f, 1.0f, 8.0f);
            ImGui::DragFloat("BBOX width in OBB proximity", &user_settings.point_cloud.bbox_width_in_obb_proximity, 0.25f, 1.0f, 8.0f);

            ImGui::Checkbox("Enable draw BBOX (master)", &user_settings.point_cloud.draw_enable_bbox);
            // ImGui::BeginDisabled(!user_settings.point_cloud.draw_enable_bbox);
            {
                ImGui::Checkbox("Enable draw BBOX (outside)", &user_settings.point_cloud.draw_enable_bbox_out);
                ImGui::Checkbox("Enable draw BBOX (in OBB)", &user_settings.point_cloud.draw_enable_bbox_in_obb);
                ImGui::Checkbox("Enable draw BBOX (in OBB proximity)", &user_settings.point_cloud.draw_enable_bbox_in_obb_proximity);
            }
            // ImGui::EndDisabled();

            ImGui::Checkbox("Enable draw PC (master)", &user_settings.point_cloud.draw_enable_pc);
            // ImGui::BeginDisabled(!user_settings.point_cloud.draw_enable_pc);
            {
                ImGui::Checkbox("Enable draw PC (outside)", &user_settings.point_cloud.draw_enable_pc_out);
                ImGui::Checkbox("Enable draw PC (in OBB)", &user_settings.point_cloud.draw_enable_pc_in_obb);
                ImGui::Checkbox("Enable draw PC (in OBB proximity)", &user_settings.point_cloud.draw_enable_pc_in_obb_proximity);
            }
            // ImGui::EndDisabled();

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
          {"draw_orientations", user_settings.trajectory.draw_orientations},
          {"orientation_axis_length", user_settings.trajectory.orientation_axis_length},
          {"width", user_settings.trajectory.width},
          {"point_size", user_settings.trajectory.point_size},
          {"display_mode", static_cast<int32_t>(user_settings.trajectory.display_mode)},
          {"color", Vec3ToJSON(user_settings.trajectory.color)}}},
        {"measurements", {{"draw_enable", user_settings.measurements.draw_enable}}},
        {"picking",
         {{"trajectory", PickToleranceToJSON(user_settings.picking.trajectory)},
          {"point_snap", PickToleranceToJSON(user_settings.picking.point_snap)},
          {"measurement", PickToleranceToJSON(user_settings.picking.measurement)}}},
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
          {"display_mode", static_cast<int32_t>(user_settings.point_cloud.display_mode)},
          {"colormap", static_cast<int32_t>(user_settings.point_cloud.colormap)}}},
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

    const json* opengl       = FindObject(settings, "opengl");
    const json* io           = FindObject(settings, "io");
    const json* origin       = FindObject(settings, "origin");
    const json* target       = FindObject(settings, "target");
    const json* trajectory   = FindObject(settings, "trajectory");
    const json* measurements = FindObject(settings, "measurements");
    const json* picking      = FindObject(settings, "picking");
    const json* stretcher    = FindObject(settings, "stretcher");
    const json* point_cloud  = FindObject(settings, "point_cloud");
    const json* collision    = FindObject(settings, "collision");

    // A helper that logs a warning when a setting cannot be read and leaves
    // the output at its already-initialised default value.
    const auto TryRead = [&path](const json* section, const char* section_name, const char* key, auto& out)
    {
        if (!section || !ReadSetting(*section, key, out))
            spdlog::warn("UserSettings ({}): failed to load {}.{}, using default", path.string(), section_name, key);
    };

    UserSettings loaded;

    // opengl
    Vec3FromJSON(opengl, "clear_color", loaded.opengl.clear_color);

    // io
    TryRead(io, "io", "map_load_extent", loaded.io.map_load_extent);
    TryRead(io, "io", "map_load_decimation_factor", loaded.io.map_load_decimation_factor);
    TryRead(io, "io", "map_load_decimation_levels", loaded.io.map_load_decimation_levels);
    TryRead(io, "io", "map_load_minimum_first_level_points", loaded.io.map_load_minimum_first_level_points);
    TryRead(io, "io", "map_load_use_center_extent", loaded.io.map_load_use_center_extent);
    TryRead(io, "io", "trajectory_load_every_nth", loaded.io.trajectory_load_every_nth);

    // origin
    TryRead(origin, "origin", "draw_enable", loaded.origin.draw_enable);
    TryRead(origin, "origin", "scale", loaded.origin.scale);
    TryRead(origin, "origin", "width", loaded.origin.width);

    // target
    TryRead(target, "target", "draw_enable", loaded.target.draw_enable);
    TryRead(target, "target", "scale", loaded.target.scale);
    TryRead(target, "target", "width", loaded.target.width);
    Vec3FromJSON(target, "color", loaded.target.color);

    // trajectory
    TryRead(trajectory, "trajectory", "draw_enable", loaded.trajectory.draw_enable);
    TryRead(trajectory, "trajectory", "width", loaded.trajectory.width);
    TryRead(trajectory, "trajectory", "point_size", loaded.trajectory.point_size);
    Vec3FromJSON(trajectory, "color", loaded.trajectory.color);

    if (trajectory && trajectory->contains("draw_orientations"))
        TryRead(trajectory, "trajectory", "draw_orientations", loaded.trajectory.draw_orientations);

    if (trajectory && trajectory->contains("orientation_axis_length"))
    {
        TryRead(trajectory, "trajectory", "orientation_axis_length", loaded.trajectory.orientation_axis_length);
        if (!(loaded.trajectory.orientation_axis_length > 0.0f) || loaded.trajectory.orientation_axis_length > 100.0f)
        {
            spdlog::warn("UserSettings ({}): trajectory.orientation_axis_length out of range, using default",
                         path.string());
            loaded.trajectory.orientation_axis_length = UserSettings{}.trajectory.orientation_axis_length;
        }
    }

    {
        int32_t display_mode_value = static_cast<int32_t>(loaded.trajectory.display_mode);
        TryRead(trajectory, "trajectory", "display_mode", display_mode_value);
        if (display_mode_value < 0 || display_mode_value > 1)
        {
            spdlog::warn("UserSettings ({}): trajectory.display_mode value {} out of range, using default",
                         path.string(), display_mode_value);
        }
        else
        {
            loaded.trajectory.display_mode = static_cast<TrajectoryDisplayMode>(display_mode_value);
        }
    }

    // measurements
    TryRead(measurements, "measurements", "draw_enable", loaded.measurements.draw_enable);

    // picking : the whole section is optional, files saved before it was introduced keep the defaults
    if (picking)
    {
        const auto LoadPickTolerance = [&](const char* name, PointPickTolerance& out)
        {
            const std::string        section_name = std::string("picking.") + name;
            const json*              section      = FindObject(*picking, name);
            const PointPickTolerance defaults     = out;

            int32_t mode_value = static_cast<int32_t>(out.mode);
            TryRead(section, section_name.c_str(), "mode", mode_value);
            if (mode_value < 0 || mode_value > static_cast<int32_t>(PickToleranceMode::PICK_TOLERANCE_MODE_LARGEST))
            {
                spdlog::warn("UserSettings ({}): {}.mode value {} out of range, using default",
                             path.string(), section_name, mode_value);
            }
            else
            {
                out.mode = static_cast<PickToleranceMode>(mode_value);
            }

            TryRead(section, section_name.c_str(), "radius_m", out.radius_m);
            if (!(out.radius_m >= PICK_RADIUS_M_MIN && out.radius_m <= PICK_RADIUS_M_MAX))
            {
                spdlog::warn("UserSettings ({}): {}.radius_m out of range, using default", path.string(), section_name);
                out.radius_m = defaults.radius_m;
            }

            TryRead(section, section_name.c_str(), "radius_px", out.radius_px);
            if (!(out.radius_px >= PICK_RADIUS_PX_MIN && out.radius_px <= PICK_RADIUS_PX_MAX))
            {
                spdlog::warn("UserSettings ({}): {}.radius_px out of range, using default", path.string(), section_name);
                out.radius_px = defaults.radius_px;
            }
        };

        LoadPickTolerance("trajectory", loaded.picking.trajectory);
        LoadPickTolerance("point_snap", loaded.picking.point_snap);
        LoadPickTolerance("measurement", loaded.picking.measurement);
    }

    // stretcher
    TryRead(stretcher, "stretcher", "draw_enable", loaded.stretcher.draw_enable);
    TryRead(stretcher, "stretcher", "draw_enable_bbox", loaded.stretcher.draw_enable_bbox);
    TryRead(stretcher, "stretcher", "bbox_width", loaded.stretcher.bbox_width);
    Vec3FromJSON(stretcher, "bbox_color", loaded.stretcher.bbox_color);

    // point_cloud
    TryRead(point_cloud, "point_cloud", "point_size", loaded.point_cloud.point_size);
    TryRead(point_cloud, "point_cloud", "bbox_width", loaded.point_cloud.bbox_width);
    TryRead(point_cloud, "point_cloud", "bbox_width_in_obb", loaded.point_cloud.bbox_width_in_obb);
    TryRead(point_cloud, "point_cloud", "bbox_width_in_obb_proximity", loaded.point_cloud.bbox_width_in_obb_proximity);
    TryRead(point_cloud, "point_cloud", "draw_enable_bbox", loaded.point_cloud.draw_enable_bbox);
    TryRead(point_cloud, "point_cloud", "draw_enable_bbox_out", loaded.point_cloud.draw_enable_bbox_out);
    TryRead(point_cloud, "point_cloud", "draw_enable_bbox_in_obb", loaded.point_cloud.draw_enable_bbox_in_obb);
    TryRead(point_cloud, "point_cloud", "draw_enable_bbox_in_obb_proximity", loaded.point_cloud.draw_enable_bbox_in_obb_proximity);
    TryRead(point_cloud, "point_cloud", "draw_enable_pc", loaded.point_cloud.draw_enable_pc);
    TryRead(point_cloud, "point_cloud", "draw_enable_pc_out", loaded.point_cloud.draw_enable_pc_out);
    TryRead(point_cloud, "point_cloud", "draw_enable_pc_in_obb", loaded.point_cloud.draw_enable_pc_in_obb);
    TryRead(point_cloud, "point_cloud", "draw_enable_pc_in_obb_proximity", loaded.point_cloud.draw_enable_pc_in_obb_proximity);

    {
        int32_t display_mode_value = static_cast<int32_t>(loaded.point_cloud.display_mode);
        TryRead(point_cloud, "point_cloud", "display_mode", display_mode_value);
        if (display_mode_value < 0 || display_mode_value > 4)
        {
            spdlog::warn("UserSettings ({}): point_cloud.display_mode value {} out of range, using default",
                         path.string(), display_mode_value);
        }
        else
        {
            loaded.point_cloud.display_mode = static_cast<PointCloudDisplayMode>(display_mode_value);
        }
    }

    {
        int32_t colormap_value = static_cast<int32_t>(loaded.point_cloud.colormap);
        TryRead(point_cloud, "point_cloud", "colormap", colormap_value);
        if (colormap_value < 0 || colormap_value > 4)
        {
            spdlog::warn("UserSettings ({}): point_cloud.colormap value {} out of range, using default",
                         path.string(), colormap_value);
        }
        else
        {
            loaded.point_cloud.colormap = static_cast<ColorMapType>(colormap_value);
        }
    }

    // collision
    TryRead(collision, "collision", "radious", loaded.collision.radious);
    TryRead(collision, "collision", "points_size", loaded.collision.points_size);
    Vec3FromJSON(collision, "points_color", loaded.collision.points_color);

    user_settings_out = loaded;
    return true;
}
