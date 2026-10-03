#include <Core/Debug.h>

#include <glad/glad.h>

#include <imgui.h>

#include <cstdint>

void DebugImGUI(PointCloudBucket& buckets, bool& open)
{
    if (ImGui::Begin("Debug", &open))
    {
        if (ImGui::TreeNode("Performance"))
        {
            ImGui::Text("Framerate  : %.3f FPS", ImGui::GetIO().Framerate);
            ImGui::Text("Frame time : %.3f ms", ImGui::GetIO().DeltaTime * 1000.0f);
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("OpenGL"))
        {
            ImGui::Text("OpenGL version      : %s", (const char*)glGetString(GL_VERSION));
            ImGui::Text("OpenGL vendor       : %s", (const char*)glGetString(GL_VENDOR));
            ImGui::Text("OpenGL renderer     : %s", (const char*)glGetString(GL_RENDERER));
            ImGui::Text("OpenGL GLSL version : %s", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
            ImGui::TreePop();
        }
        ImGui::Separator();

        if (ImGui::TreeNode("Buckets"))
        {
            if (ImGui::Button("Set all draw ON"))
            {
                for (auto& [ID, bucket] : buckets)
                {
                    bucket.draw = true;
                }
            }

            if (ImGui::Button("Set all draw OFF"))
            {
                for (auto& [ID, bucket] : buckets)
                {
                    bucket.draw = false;
                }
            }

            if (ImGui::TreeNode("Cave buckets"))
            {
                ImGui::Text("buckets = %zu", buckets.size());

                for (auto& [ID, bucket] : buckets)
                {
                    if (ImGui::TreeNode(&ID, "[%d, %d, %d]", ID.x, ID.y, ID.z))
                    {
                        ImGui::Checkbox("draw", &bucket.draw);

                        ImGui::Text("aabb.min : %.3f, %.3f, %.3f", bucket.aabb.min.x, bucket.aabb.min.y, bucket.aabb.min.z);
                        ImGui::Text("aabb.max   : %.3f, %.3f, %.3f", bucket.aabb.max.x, bucket.aabb.max.y, bucket.aabb.max.z);

                        PointCloudLOD* current = bucket.lods;
                        int            lod     = 0;
                        while (current)
                        {
                            if (ImGui::TreeNode((void*)(intptr_t)lod, "LOD %d", lod))
                            {
                                ImGui::Text("min : %.3f, %.3f, %.3f", current->min.x, current->min.y, current->min.z);
                                ImGui::Text("max : %.3f, %.3f, %.3f", current->max.x, current->max.y, current->max.z);
                                ImGui::Text("VAO = %u, VBO = %u, points = %zu", current->vao->GetID(), current->vbo->GetID(), current->points.size());

                                ImGui::TreePop();
                            }

                            current = current->next;
                            ++lod;
                        }

                        ImGui::TreePop();
                    }
                }
                ImGui::TreePop();
            }

            ImGui::TreePop();
        }
    }
    ImGui::End();
}
