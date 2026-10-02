#include <cave-traversal-tool/ProjectGui.h>

#include <imgui.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <limits>

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

                ImGui::Checkbox(("draw_axes_overlay##" + std::to_string(i)).c_str(), &ctx.draw_axes_overlay[i]);
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Show world axes orientation overlay in this viewport");
                }

                ImGui::BeginDisabled(!ctx.draw_axes_overlay[i]);
                ImGui::DragFloat(("axes_overlay_size##" + std::to_string(i)).c_str(), &ctx.axes_overlay_size[i], 0.001f, 0.025f, 0.15f, "%.3f");
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Overlay size as a fraction of the viewport (0.02 to 0.5)");
                }
                ImGui::EndDisabled();

                ImGui::Checkbox(("draw_measurement_labels##" + std::to_string(i)).c_str(), &ctx.draw_measurement_labels[i]);
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Show measurement value labels in this viewport");
                }

                // Per-camera FOV (all viewports) : perspective projection only
                ImGui::BeginDisabled(ctx.cameras[i].projection_type != ProjectionType::PROJECTION_TYPE_PERSPECTIVE);
                if (ImGui::DragFloat(("fov_y##" + std::to_string(i)).c_str(), &ctx.cameras[i].fov_y, 0.5f, 1.0f, 170.0f, "%.1f deg"))
                {
                    ctx.cameras[i].fov_y = std::clamp(ctx.cameras[i].fov_y, 1.0f, 170.0f);
                }
                ImGui::EndDisabled();

                // Per-viewport LOD control
                const int32_t max_lod_index = project_data.max_lod_count > 0 ? static_cast<int32_t>(project_data.max_lod_count - 1) : 0;
                ImGui::Checkbox(("use_fixed_lod##" + std::to_string(i)).c_str(), &ctx.use_fixed_lod[i]);
                ImGui::BeginDisabled(!ctx.use_fixed_lod[i]);
                ImGui::SliderInt(("fixed_lod_index##" + std::to_string(i)).c_str(), &ctx.fixed_lod_index[i], 0, max_lod_index);
                ImGui::EndDisabled();

                // Per-viewport projection type : perspective (fov) or orthographic (parallel box)
                {
                    const char* projection_names[] = {"Perspective", "Orthographic"};
                    int         projection         = static_cast<int>(ctx.cameras[i].projection_type);
                    if (ImGui::Combo(("projection##" + std::to_string(i)).c_str(), &projection, projection_names, 2))
                    {
                        ctx.cameras[i].projection_type = static_cast<ProjectionType>(projection);

                        // On change just reset the planes to a sane range and let the
                        // user adjust them with the sliders below (both projection types).
                        ctx.cameras[i].near_plane = 0.01f;
                        ctx.cameras[i].far_plane  = 1000.0f;
                    }
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Camera projection : perspective (angle dependent) or orthographic (parallel)");
                    }
                }

                int mode = static_cast<int>(ctx.camera_modes[i]);
                if (i >= 1 && ImGui::Combo(("##camera_mode_" + std::to_string(i)).c_str(), &mode, camera_mode_names, 13))
                {
                    CameraMode old_mode = ctx.camera_modes[i];
                    ctx.camera_modes[i] = static_cast<CameraMode>(mode);
                    if (ctx.camera_modes[i] != CameraMode::FREE_ORBIT && old_mode != ctx.camera_modes[i])
                    {
                        // Entering a non free look camera : calculate planes from the view distance
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
                        // Back to free look : reset planes for manual control
                        ctx.cameras[i].near_plane = 0.01f;
                        ctx.cameras[i].far_plane  = 1000.0f;
                        unlock_camera_to_free_orbit(ctx.cameras[i], ctx.view_axis_distance[i]);
                    }
                }
                ImGui::SameLine();
                ImGui::Text("camera");

                const bool locked   = (i >= 1 && ctx.camera_modes[i] != CameraMode::FREE_ORBIT);
                const bool is_ortho = (ctx.cameras[i].projection_type == ProjectionType::PROJECTION_TYPE_ORTHOGRAPHIC);

                if (locked)
                {
                    // Non free look camera : planes are calculated each frame from the view distance,
                    // so they are shown read-only and only the distance / offset can be edited here.
                    const char* plane_mode_names[] = {"Symmetrical", "Asymmetrical"};
                    int         plane_mode         = ctx.symmetric_planes[i] ? 0 : 1;
                    if (ImGui::Combo(("plane_mode##" + std::to_string(i)).c_str(), &plane_mode, plane_mode_names, 2))
                    {
                        ctx.symmetric_planes[i] = (plane_mode == 0);
                    }

                    ImGui::DragFloat(("plane_distance##" + std::to_string(i)).c_str(), &ctx.view_axis_distance[i], 0.1f, 0.1f, FLT_MAX, "%.3f");

                    if (ctx.symmetric_planes[i])
                    {
                        ImGui::DragFloat(("plane_offset##" + std::to_string(i)).c_str(), &ctx.symmetric_plane_offset[i], 0.05f, 0.05f, 100.0f, "%.3f");

                        if (ImGui::Button(("Reset offset (1.25m)##" + std::to_string(i)).c_str()))
                        {
                            ctx.symmetric_plane_offset[i] = 1.25f;
                        }
                    }

                    ImGui::BeginDisabled(true);
                    ImGui::DragFloat(("near_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].near_plane, 0.05f, 0.01f, 1000.0f, "%.3f");
                    ImGui::DragFloat(("far_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].far_plane, 0.05f, 0.01f, 1000.0f, "%.3f");
                    if (is_ortho)
                    {
                        // Orthographic : the view axis distance is used as a dolly, so the box
                        // half height is derived from it and shown read-only
                        ImGui::DragFloat(("ortho_half_height##" + std::to_string(i)).c_str(), &ctx.cameras[i].ortho_half_height, 0.05f, 0.01f, 10000.0f, "%.3f");
                    }
                    ImGui::EndDisabled();
                }
                else if (is_ortho)
                {
                    // Free orbit : orthographic box - view-space half height + 0.01..1000 depth range
                    ImGui::DragFloat(("ortho_half_height##" + std::to_string(i)).c_str(), &ctx.cameras[i].ortho_half_height, 0.05f, 0.01f, 10000.0f, "%.3f");
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Half of the vertical world extent visible in this viewport");
                    }

                    ImGui::DragFloat(("ortho_zoom##" + std::to_string(i)).c_str(), &ctx.cameras[i].ortho_zoom, 0.01f, 0.01f, 100.0f, "%.3f");
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Orthographic zoom : > 1 magnifies, < 1 zooms out (also changed with the scroll wheel)");
                    }

                    ImGui::DragFloat(("near_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].near_plane, 0.05f, 0.01f, 1000.0f, "%.3f");
                    ImGui::DragFloat(("far_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].far_plane, 0.05f, 0.01f, 1000.0f, "%.3f");
                }
                else
                {
                    // Free orbit (and viewport 0) : independent near / far plane control, 0.01..1000
                    ImGui::DragFloat(("near_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].near_plane, 0.05f, 0.01f, 1000.0f, "%.3f");
                    ImGui::DragFloat(("far_plane##" + std::to_string(i)).c_str(), &ctx.cameras[i].far_plane, 0.05f, 0.01f, 1000.0f, "%.3f");
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

                ImGui::Separator();
                ImGui::Checkbox("Auto Play", &project_data.trajectory_index_auto_play);
                ImGui::DragInt("Auto Play increment", &project_data.trajectory_index_auto_play_increment, 1.0f, 1, INT32_MAX);

                ImGui::Separator();
                ImGui::DragScalar("Index (fine adjustment)", ImGuiDataType_U32, &project_data.trajectory_index, 1.0f, &zero, &max_orientation_index);
                ImGui::SliderScalar("Index (coarse adjustment)", ImGuiDataType_U32, &project_data.trajectory_index, &zero, &max_orientation_index);

                ImGui::Separator();
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
        if (ImGui::TreeNode("Measurements"))
        {
            MeasurementState& ms = project_data.measurements;

            // Status line
            if (!user_settings.measurements.draw_enable)
            {
                ImGui::TextColored({1.0f, 0.5f, 0.0f, 1.0f}, "Measurement display and picking are disabled in UserSettings");
            }
            else if (ms.pending_point.has_value())
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

                if (ImGui::BeginTable("##meas_table", 7,
                                      ImGuiTableFlags_Borders |
                                          ImGuiTableFlags_RowBg |
                                          ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 28.0f);
                    ImGui::TableSetupColumn("Point A", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Point B", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Distance (m)", ImGuiTableColumnFlags_WidthFixed, 110.0f);
                    ImGui::TableSetupColumn("Color", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                    ImGui::TableSetupColumn("Width", ImGuiTableColumnFlags_WidthFixed, 75.0f);
                    ImGui::TableSetupColumn("##del", ImGuiTableColumnFlags_WidthFixed, 26.0f);
                    ImGui::TableHeadersRow();

                    for (int i = 0; i < static_cast<int>(ms.entries.size()); ++i)
                    {
                        MeasurementEntry& e = ms.entries[i];
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
                        // ColorEdit3 expects a float[3] it can write through directly;
                        // edit a local copy and copy back to keep the swatch in sync with the value
                        float meas_color[3] = {e.color.x, e.color.y, e.color.z};
                        if (ImGui::ColorEdit3("##meas_color", meas_color, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel))
                        {
                            e.color = glm::vec3(meas_color[0], meas_color[1], meas_color[2]);
                        }
                        ImGui::PopID();

                        ImGui::TableSetColumnIndex(5);
                        ImGui::PushID(i);
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::DragFloat("##meas_width", &e.line_width, 0.25f, 1.0f, 8.0f, "%.1f"))
                        {
                            e.line_width = std::clamp(e.line_width, 1.0f, 8.0f);
                        }
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::SetTooltip("Line width (1 to 8, default 2)");
                        }
                        ImGui::PopID();

                        ImGui::TableSetColumnIndex(6);
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
