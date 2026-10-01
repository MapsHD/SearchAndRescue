#include <cave-traversal-tool/ProjectGui.h>

#include <imgui.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <limits>

// Moves camera target (keeping position - target offset) to the given pose position
static inline void snap_camera_target_to_trajectory(Camera& cam, const glm::vec3& pose_pos)
{
    const glm::vec3 offset = cam.position - cam.target;
    cam.target             = pose_pos;
    cam.position           = pose_pos + offset;
}

void ProjectDataImGUI(ProjectData& project_data, const UserSettings& user_settings, bool& open)
{
    MultiViewContext& ctx = project_data.multi_view;

    if (ImGui::Begin("ProjectData", &open))
    {
        if (ImGui::TreeNode("File input / output"))
        {
            if (ImGui::Button("Load trajectory", ImVec2(200.0f, 0.0f)))
            {
                load_trajectory_dialog(project_data, user_settings);
            }
            ImGui::Text("%s", project_data.trajectory_path.empty() ? "(none)" : project_data.trajectory_path.c_str());

            ImGui::Separator();

            if (ImGui::Button("Load object", ImVec2(200.0f, 0.0f)))
            {
                load_object_dialog(project_data);
            }
            ImGui::Text("%s", project_data.object_path.empty() ? "(none)" : project_data.object_path.c_str());

            ImGui::Separator();

            if (ImGui::Button("Load environment", ImVec2(200.0f, 0.0f)))
            {
                load_environment_dialog(project_data, user_settings);
            }
            ImGui::Text("%s", project_data.environment_path.empty() ? "(none)" : project_data.environment_path.c_str());

            ImGui::TreePop();
        }

        ImGui::Separator();
        if (ImGui::TreeNode("Viewport"))
        {
            int count = static_cast<int>(ctx.active_count);
            if (ImGui::RadioButton("1 (single)", &count, 1))
            {
                ctx.active_count = ViewportCount::ONE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("2 (L/R split)", &count, 2))
            {
                ctx.active_count = ViewportCount::TWO;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("4 (2x2 grid)", &count, 4))
            {
                ctx.active_count = ViewportCount::FOUR;
            }

            static const char* camera_mode_names[] = {"Free", "+X", "-X", "+Y", "-Y", "+Z", "-Z", "+local X", "-local X", "+local Y", "-local Y", "+local Z", "-local Z"};

            for (int i = 0; i <= 3; ++i)
            {
                if (i >= static_cast<int>(ctx.active_count))
                {
                    continue;
                }

                ImGui::Separator();
                ImGui::Text("Viewport %d", i + 1);

                // Per-camera FOV (all viewports)
                if (ImGui::DragFloat(("fov_y##" + std::to_string(i)).c_str(), &ctx.cameras[i].fov_y, 0.5f, 1.0f, 170.0f, "%.1f deg"))
                {
                    ctx.cameras[i].fov_y = std::clamp(ctx.cameras[i].fov_y, 1.0f, 170.0f);
                }

                // Per-viewport LOD control
                const int32_t max_lod_index = project_data.max_lod_count > 0 ? static_cast<int32_t>(project_data.max_lod_count - 1) : 0;
                ImGui::Checkbox(("use_fixed_lod##" + std::to_string(i)).c_str(), &ctx.use_fixed_lod[i]);
                ImGui::BeginDisabled(!ctx.use_fixed_lod[i]);
                ImGui::SliderInt(("fixed_lod_index##" + std::to_string(i)).c_str(), &ctx.fixed_lod_index[i], 0, max_lod_index);
                ImGui::EndDisabled();

                int mode = static_cast<int>(ctx.camera_modes[i]);
                if (i >= 1 && ImGui::Combo(("##camera_mode_" + std::to_string(i)).c_str(), &mode, camera_mode_names, 13))
                {
                    CameraMode old_mode = ctx.camera_modes[i];
                    ctx.camera_modes[i] = static_cast<CameraMode>(mode);
                    if (ctx.camera_modes[i] != CameraMode::FREE_ORBIT && old_mode != ctx.camera_modes[i])
                    {
                        if (ctx.symmetric_planes[i])
                        {
                            ctx.cameras[i].near_plane = std::max(0.01f, ctx.view_axis_distance[i] - ctx.symmetric_plane_offset[i]);
                            ctx.cameras[i].far_plane  = ctx.view_axis_distance[i] + ctx.symmetric_plane_offset[i];
                        }
                        else
                        {
                            ctx.cameras[i].near_plane = std::max(0.01f, ctx.view_axis_distance[i] - 1.5f);
                            ctx.cameras[i].far_plane  = ctx.view_axis_distance[i] + 1.5f;
                        }
                    }
                    else if (ctx.camera_modes[i] == CameraMode::FREE_ORBIT && old_mode != CameraMode::FREE_ORBIT)
                    {
                        ctx.cameras[i].near_plane = 0.1f;
                        ctx.cameras[i].far_plane  = 1000.0f;
                        unlock_camera_to_free_orbit(ctx.cameras[i], ctx.view_axis_distance[i]);
                    }
                }
                ImGui::SameLine();
                ImGui::Text("camera");

                if (i >= 1 && ctx.camera_modes[i] != CameraMode::FREE_ORBIT)
                {
                    // Plane control mode : symmetric (single slider, planes derived from distance) vs asymmetric (independent planes)
                    const char* plane_mode_names[] = {"Symmetrical", "Asymmetrical"};
                    int         plane_mode         = ctx.symmetric_planes[i] ? 0 : 1;
                    if (ImGui::Combo(("plane_mode##" + std::to_string(i)).c_str(), &plane_mode, plane_mode_names, 2))
                    {
                        ctx.symmetric_planes[i] = (plane_mode == 0);

                        if (ctx.symmetric_planes[i])
                        {
                            ctx.cameras[i].near_plane = std::max(0.01f, ctx.view_axis_distance[i] - ctx.symmetric_plane_offset[i]);
                            ctx.cameras[i].far_plane  = ctx.view_axis_distance[i] + ctx.symmetric_plane_offset[i];
                        }
                    }

                    if (ctx.symmetric_planes[i])
                    {
                        // Symmetrical : single slider controls camera distance from the stretcher pose,
                        // planes are enforced each frame as distance -+ symmetric_plane_offset
                        ImGui::DragFloat(("plane_distance##" + std::to_string(i)).c_str(), &ctx.view_axis_distance[i], 0.1f, 0.1f, FLT_MAX, "%.3f");

                        // ImGui::BeginDisabled(true);
                        // ImGui::DragFloat(("near_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].near_plane, 0.05f, 0.01f, 10000.0f, "%.3f");
                        // ImGui::DragFloat(("far_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].far_plane, 0.05f, 0.01f, 10000.0f, "%.3f");
                        // ImGui::EndDisabled();

                        ImGui::DragFloat(("plane_offset##" + std::to_string(i)).c_str(), &ctx.symmetric_plane_offset[i], 0.05f, 0.05f, 100.0f, "%.3f");

                        if (ImGui::Button(("Reset offset (1.25m)##" + std::to_string(i)).c_str()))
                        {
                            ctx.symmetric_plane_offset[i] = 1.25f;
                        }
                    }
                    else
                    {
                        // Asymmetrical : independent distance / near / far control (previous behaviour)
                        float old_dist = ctx.view_axis_distance[i];
                        if (ImGui::DragFloat(("view_axis_distance##" + std::to_string(i)).c_str(), &ctx.view_axis_distance[i], 0.1f, 0.1f, FLT_MAX, "%.3f"))
                        {
                            float delta               = ctx.view_axis_distance[i] - old_dist;
                            ctx.cameras[i].near_plane = std::max(0.01f, ctx.cameras[i].near_plane + delta);
                            ctx.cameras[i].far_plane  = std::max(ctx.cameras[i].near_plane + 0.05f, ctx.cameras[i].far_plane + delta);
                        }

                        ImGui::DragFloat(("near_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].near_plane, 0.05f, 0.01f, ctx.cameras[i].far_plane - 0.01f, "%.3f");
                        ImGui::DragFloat(("far_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].far_plane, 0.05f, ctx.cameras[i].near_plane + 0.01f, 10000.0f, "%.3f");

                        if (ctx.cameras[i].near_plane < 0.01f)
                        {
                            ctx.cameras[i].near_plane = 0.01f;
                        }
                        if (ctx.cameras[i].far_plane <= ctx.cameras[i].near_plane)
                        {
                            ctx.cameras[i].far_plane = ctx.cameras[i].near_plane + 0.05f;
                        }

                        if (ImGui::Button(("Reset planes (+-1m)##" + std::to_string(i)).c_str()))
                        {
                            ctx.cameras[i].near_plane = std::max(0.01f, ctx.view_axis_distance[i] - 1.0f);
                            ctx.cameras[i].far_plane  = ctx.view_axis_distance[i] + 1.0f;
                        }
                    }
                }
                else
                {
                    // Free orbit (and viewport 0) : independent near / far plane control
                    ImGui::DragFloat(("near_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].near_plane, 0.05f, 0.01f, ctx.cameras[i].far_plane - 0.01f, "%.3f");
                    ImGui::DragFloat(("far_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].far_plane, 0.05f, ctx.cameras[i].near_plane + 0.01f, 10000.0f, "%.3f");

                    if (ctx.cameras[i].near_plane < 0.01f)
                    {
                        ctx.cameras[i].near_plane = 0.01f;
                    }
                    if (ctx.cameras[i].far_plane <= ctx.cameras[i].near_plane)
                    {
                        ctx.cameras[i].far_plane = ctx.cameras[i].near_plane + 0.05f;
                    }
                }
            }

            ImGui::TreePop();
        }

        ImGui::Separator();
        if (ImGui::TreeNode("Trajectory"))
        {
            if (project_data.trajectory_orientations_mat33.size())
            {
                const uint32_t zero                  = 0U;
                const uint32_t max_orientation_index = static_cast<uint32_t>(project_data.trajectory_orientations_mat33.size()) - 1U;

                ImGui::Text("Trajectory : %zu / %zu = %.2f%", static_cast<size_t>(project_data.trajectory_index), static_cast<size_t>(max_orientation_index), static_cast<float>(project_data.trajectory_index) / static_cast<float>(max_orientation_index) * 100.0f);
                ImGui::Checkbox("trajectory_index_auto_play", &project_data.trajectory_index_auto_play);
                ImGui::DragInt("trajectory_index_auto_play_increment", &project_data.trajectory_index_auto_play_increment, 1.0f, 1, INT32_MAX);
                ImGui::DragScalar("trajectory_index", ImGuiDataType_U32, &project_data.trajectory_index, 1.0f, &zero, &max_orientation_index);

                // Trajectory traversal buttons : move +- given meters along the trajectory
                if (ImGui::Button("- 1 m"))
                {
                    move_trajectory_index_by_distance(project_data.trajectory_positions, project_data.trajectory_index, -1.0f);
                }
                ImGui::SameLine();
                if (ImGui::Button("- 5 m"))
                {
                    move_trajectory_index_by_distance(project_data.trajectory_positions, project_data.trajectory_index, -5.0f);
                }
                ImGui::SameLine();
                if (ImGui::Button("- 10 m"))
                {
                    move_trajectory_index_by_distance(project_data.trajectory_positions, project_data.trajectory_index, -10.0f);
                }

                if (ImGui::Button("+ 1 m"))
                {
                    move_trajectory_index_by_distance(project_data.trajectory_positions, project_data.trajectory_index, +1.0f);
                }
                ImGui::SameLine();
                if (ImGui::Button("+ 5 m"))
                {
                    move_trajectory_index_by_distance(project_data.trajectory_positions, project_data.trajectory_index, +5.0f);
                }
                ImGui::SameLine();
                if (ImGui::Button("+ 10 m"))
                {
                    move_trajectory_index_by_distance(project_data.trajectory_positions, project_data.trajectory_index, +10.0f);
                }
            }
            else
            {
                ImGui::TextColored({1.0f, 0.0f, 0.0f, 1.0f}, "Can not set index of trajectory pose - load trajectory!");
            }

            ImGui::TreePop();
        }

        ImGui::Separator();
        ImGui::Checkbox("Lock target to trajectory", &project_data.lock_viewport0_target_to_trajectory);
        ImGui::SameLine();
        if (ImGui::Button("Snap"))
        {
            if (project_data.trajectory_positions.size())
            {
                snap_camera_target_to_trajectory(project_data.multi_view.cameras[0], project_data.trajectory_positions[project_data.trajectory_index].position);
            }
        }
        if (project_data.lock_viewport0_target_to_trajectory)
        {
            ImGui::TextDisabled("locked - viewport 0 target follows current trajectory pose each frame");
        }

        ImGui::Separator();
        if (ImGui::TreeNode("Measurements"))
        {
            MeasurementState& ms = project_data.measurements;

            // Status line
            if (ms.pending_point.has_value())
            {
                const glm::vec3& p = ms.pending_point.value();
                ImGui::TextColored({1.0f, 1.0f, 0.0f, 1.0f},
                                   "Pending: (%.3f, %.3f, %.3f)  -- Shift+LMB to pick end point",
                                   p.x, p.y, p.z);
            }
            else
            {
                ImGui::TextDisabled("Shift+LMB in any viewport to start a measurement");
            }

            ImGui::Spacing();

            // Measurement table
            if (ms.entries.empty())
            {
                ImGui::TextDisabled("(no measurements yet)");
            }
            else
            {
                int erase_index = -1; // deferred single-row deletion

                if (ImGui::BeginTable("##meas_table", 5,
                                      ImGuiTableFlags_Borders |
                                          ImGuiTableFlags_RowBg |
                                          ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 28.0f);
                    ImGui::TableSetupColumn("Point A", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Point B", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Distance (m)", ImGuiTableColumnFlags_WidthFixed, 110.0f);
                    ImGui::TableSetupColumn("##del", ImGuiTableColumnFlags_WidthFixed, 26.0f);
                    ImGui::TableHeadersRow();

                    for (int i = 0; i < static_cast<int>(ms.entries.size()); ++i)
                    {
                        const MeasurementEntry& e = ms.entries[i];
                        ImGui::TableNextRow();

                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%d", i + 1);

                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("(%.3f, %.3f, %.3f)", e.point_a.x, e.point_a.y, e.point_a.z);

                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("(%.3f, %.3f, %.3f)", e.point_b.x, e.point_b.y, e.point_b.z);

                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%.4f", e.distance_m);

                        ImGui::TableSetColumnIndex(4);
                        ImGui::PushID(i);
                        if (ImGui::SmallButton("x"))
                        {
                            erase_index = i;
                        }
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::SetTooltip("Remove measurement %d", i + 1);
                        }
                        ImGui::PopID();
                    }
                    ImGui::EndTable();
                }

                // Apply deferred deletion (outside the table loop to avoid invalidation)
                if (erase_index >= 0)
                {
                    ms.entries.erase(ms.entries.begin() + erase_index);
                }

                ImGui::Spacing();
                if (ImGui::Button("Clear all measurements"))
                {
                    ms.entries.clear();
                    ms.pending_point.reset();
                }
            }

            // Cancel pending is always reachable when a point is waiting
            if (ms.pending_point.has_value())
            {
                ImGui::SameLine();
                if (ImGui::Button("Cancel pending"))
                {
                    ms.pending_point.reset();
                }
            }

            ImGui::TreePop();
        }
    }
    ImGui::End();
}
