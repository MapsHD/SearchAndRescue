// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

#include <algorithm>
#include <cctype>
#include <filesystem>

#include <glm/glm.hpp>

#include <Core/FileIO.h>
#include <Core/OpenGL/Program.h>
#include <Core/PFDWrapper.h>
#include <Core/Processing.h>
#include <Core/Project.h>

// Free all OpenGL resources owned by the project data
void free_project_data(ProjectData& project_data)
{
    for (auto& [ID, bucket] : project_data.buckets)
    {
        // Free LODs
        PointCloudLOD* current = bucket.lods;
        while (current)
        {
            delete current->vao;
            delete current->vbo;

            PointCloudLOD* next = current->next;
            delete current;
            current = next;
        }
        bucket.lods = nullptr;

        delete bucket.bbox_vao;
        delete bucket.bbox_vbo;
    }

    project_data.buckets.clear();

    delete project_data.trajectory_axes_vao;
    delete project_data.trajectory_positions_vao;
    delete project_data.trajectory_positions_vbo;
    delete project_data.trajectory_orientations_vbo;

    delete project_data.stretcher_aabb_vao;
    delete project_data.stretcher_aabb_vbo;
    delete project_data.stretcher_vao;
    delete project_data.stretcher_vbo;
    delete project_data.stretcher_index_buffer;

    delete project_data.collision_points_vao;
    delete project_data.collision_points_vbo;
}

bool rebuild_trajectory_mat33_opengl_data(ProjectData& project_data)
{
    const std::vector<VertexBufferAttributeLayout> layout_point = opengl_vertex_array_get_vertex_layout<Point>();

    if (project_data.trajectory_positions.empty() || project_data.trajectory_orientations_mat33.empty() ||
        project_data.trajectory_positions.size() != project_data.trajectory_orientations_mat33.size())
    {
        spdlog::error("Refusing to rebuild trajectory OpenGL data : missing or mismatched trajectory poses");
        return false;
    }

    project_data.trajectory_index = 0;

    if (project_data.trajectory_axes_vao)
    {
        spdlog::debug("Deleting old trajectory axes VAO : {}", project_data.trajectory_axes_vao->GetID());
        delete project_data.trajectory_axes_vao;
        project_data.trajectory_axes_vao = nullptr;
    }

    if (project_data.trajectory_positions_vao)
    {
        spdlog::debug("Deleting old trajectory positions VAO : {}", project_data.trajectory_positions_vao->GetID());
        delete project_data.trajectory_positions_vao;
    }

    if (project_data.trajectory_positions_vbo)
    {
        spdlog::debug("Deleting old trajectory positions VBO : {}", project_data.trajectory_positions_vbo->GetID());
        delete project_data.trajectory_positions_vbo;
    }

    if (project_data.trajectory_orientations_vbo)
    {
        spdlog::debug("Deleting old trajectory orientations VBO : {}", project_data.trajectory_orientations_vbo->GetID());
        delete project_data.trajectory_orientations_vbo;
        project_data.trajectory_orientations_vbo = nullptr;
    }

    // Positions VBO + VAO (used for the plain trajectory line/point draw pass)
    project_data.trajectory_positions_vbo = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(project_data.trajectory_positions), project_data.trajectory_positions.data());
    project_data.trajectory_positions_vao = new VertexArray(project_data.trajectory_positions_vbo, false, nullptr, false, layout_point);

    // Orientation VBO: three tightly-packed vec3 columns per pose (AxisX, AxisY, AxisZ).
    // Layout in memory per vertex: [X.x X.y X.z | Y.x Y.y Y.z | Z.x Z.y Z.z]
    struct OrientationVertex
    {
        glm::vec3 axis_x;
        glm::vec3 axis_y;
        glm::vec3 axis_z;
    };

    std::vector<OrientationVertex> orientations;
    orientations.reserve(project_data.trajectory_orientations_mat33.size());
    for (const auto& pose : project_data.trajectory_orientations_mat33)
    {
        orientations.push_back({
            pose.orientation[0], // column 0 = X axis
            pose.orientation[1], // column 1 = Y axis
            pose.orientation[2], // column 2 = Z axis
        });
    }
    project_data.trajectory_orientations_vbo = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(orientations), orientations.data());

    // Combined axes VAO: positions at binding 0 (location 0),
    // orientation columns at binding 1 (locations 1, 2, 3).
    const int32_t                          orient_stride = static_cast<int32_t>(sizeof(OrientationVertex));
    const std::vector<VertexBufferBinding> axes_bindings = {
        // Positions VBO -- binding slot 0, attribute location 0
        {
            project_data.trajectory_positions_vbo,
            /* ownership */ false,
            layout_point,
            /* binding_index */ 0},
        // Orientations VBO -- binding slot 1, attributes at locations 1, 2, 3
        {
            project_data.trajectory_orientations_vbo,
            /* ownership */ false,
            std::vector<VertexBufferAttributeLayout>{
                {1, 3, /* GL_FLOAT */ 0x1406, 0, orient_stride, static_cast<int32_t>(offsetof(OrientationVertex, axis_x))},
                {2, 3, /* GL_FLOAT */ 0x1406, 0, orient_stride, static_cast<int32_t>(offsetof(OrientationVertex, axis_y))},
                {3, 3, /* GL_FLOAT */ 0x1406, 0, orient_stride, static_cast<int32_t>(offsetof(OrientationVertex, axis_z))},
            },
            /* binding_index */ 1},
    };
    project_data.trajectory_axes_vao = new VertexArray(axes_bindings, nullptr, false);

    spdlog::debug("Created trajectory positions VAO [{}] VBO [{}], orientations VBO [{}], axes VAO [{}]",
                  project_data.trajectory_positions_vao->GetID(),
                  project_data.trajectory_positions_vbo->GetID(),
                  project_data.trajectory_orientations_vbo->GetID(),
                  project_data.trajectory_axes_vao->GetID());

    return true;
}

bool rebuild_stretcher_opengl_data(ProjectData& project_data)
{
    const std::vector<VertexBufferAttributeLayout> layout_color_point = opengl_vertex_array_get_vertex_layout<ColorPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point       = opengl_vertex_array_get_vertex_layout<Point>();

    if (project_data.stretcher_vertices.empty() || project_data.stretcher_indices.empty())
    {
        spdlog::error("Refusing to rebuild stretcher OpenGL data : no stretcher data loaded");
        return false;
    }

    if (project_data.stretcher_aabb_vao)
    {
        spdlog::debug("Deleting old stretcher AABB positions VAO : {}", project_data.stretcher_aabb_vao->GetID());
        delete project_data.stretcher_aabb_vao;
    }

    if (project_data.stretcher_aabb_vbo)
    {
        spdlog::debug("Deleting old stretcher AABB positions VBO : {}", project_data.stretcher_aabb_vbo->GetID());
        delete project_data.stretcher_aabb_vbo;
    }

    if (project_data.stretcher_vao)
    {
        spdlog::debug("Deleting old stretcher positions VAO : {}", project_data.stretcher_vao->GetID());
        delete project_data.stretcher_vao;
    }

    if (project_data.stretcher_vbo)
    {
        spdlog::debug("Deleting old stretcher positions VBO : {}", project_data.stretcher_vbo->GetID());
        delete project_data.stretcher_vbo;
    }

    if (project_data.stretcher_index_buffer)
    {
        spdlog::debug("Deleting old stretcher index IBO : {}", project_data.stretcher_index_buffer->GetID());
        delete project_data.stretcher_index_buffer;
    }

    project_data.stretcher_aabb.min = project_data.stretcher_vertices[0].position;
    project_data.stretcher_aabb.max = project_data.stretcher_vertices[0].position;

    for (const auto& v : project_data.stretcher_vertices)
    {
        const glm::vec3& p = v.position;

        project_data.stretcher_aabb.min = glm::min(project_data.stretcher_aabb.min, p);
        project_data.stretcher_aabb.max = glm::max(project_data.stretcher_aabb.max, p);
    }

    std::vector<Point> line_vertices{
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.min.z}},

        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.max.z}},

        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.min.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.max.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.max.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.min.z}},
        {{project_data.stretcher_aabb.min.x, project_data.stretcher_aabb.max.y, project_data.stretcher_aabb.max.z}}};

    project_data.stretcher_aabb_vbo = new Buffer(GL_NONE, std_vector_size(line_vertices), line_vertices.data());
    project_data.stretcher_aabb_vao = new VertexArray(project_data.stretcher_aabb_vbo, false, nullptr, false, layout_point);

    project_data.stretcher_vbo          = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(project_data.stretcher_vertices), project_data.stretcher_vertices.data());
    project_data.stretcher_index_buffer = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(project_data.stretcher_indices), project_data.stretcher_indices.data());

    project_data.stretcher_vao = new VertexArray(project_data.stretcher_vbo, false, project_data.stretcher_index_buffer, false, layout_color_point);

    spdlog::debug("Created VAO [{}], VBO [{}] and IBO [{}]", project_data.stretcher_vao->GetID(), project_data.stretcher_vbo->GetID(), project_data.stretcher_index_buffer->GetID());

    return true;
}

bool rebuild_cave_opengl_data(ProjectData& project_data, const UserSettings& user_settings)
{
    const std::vector<VertexBufferAttributeLayout> layout_point_intensity = opengl_vertex_array_get_vertex_layout<PointIntensity>();
    const std::vector<VertexBufferAttributeLayout> layout_point           = opengl_vertex_array_get_vertex_layout<Point>();

    if (project_data.cave_vertices.empty())
    {
        spdlog::error("Refusing to rebuild cave OpenGL data : no cave points loaded");
        return false;
    }

    // Clear old data
    for (auto& [ID, bucket] : project_data.buckets)
    {
        // Clear LODs
        PointCloudLOD* current = bucket.lods;
        while (current)
        {
            if (current->vao)
            {
                spdlog::debug("Deleting old cave [ID = {} {} {}] VAO [{}]", ID.x, ID.y, ID.z, current->vao->GetID());
                delete current->vao;
                current->vao = nullptr;
            }

            if (current->vbo)
            {
                spdlog::debug("Deleting old cave [ID = {} {} {}] VBO [{}]", ID.x, ID.y, ID.z, current->vbo->GetID());
                delete current->vbo;
                current->vbo = nullptr;
            }

            current->points.clear();
            PointCloudLOD* next = current->next;
            delete current;
            current = next;
        }
        bucket.lods = nullptr;
        bucket.draw = false;

        if (bucket.bbox_vao)
        {
            spdlog::debug("Deleting old bounding box VAO [{}] for ID = {} {} {}", bucket.bbox_vao->GetID(), ID.x, ID.y, ID.z);
            delete bucket.bbox_vao;
            bucket.bbox_vao = nullptr;
        }

        if (bucket.bbox_vbo)
        {
            spdlog::debug("Deleting old bounding box VBO [{}] for ID = {} {} {}", bucket.bbox_vbo->GetID(), ID.x, ID.y, ID.z);
            delete bucket.bbox_vbo;
            bucket.bbox_vbo = nullptr;
        }
    }

    project_data.buckets.clear();

    project_data.intensity_min = project_data.cave_vertices.front().intensity;
    project_data.intensity_max = project_data.intensity_min;

    project_data.cave_aabb.min = project_data.cave_vertices.front().position;
    project_data.cave_aabb.max = project_data.cave_aabb.min;

    for (const auto& p : project_data.cave_vertices)
    {
        project_data.intensity_min = std::min(project_data.intensity_min, p.intensity);
        project_data.intensity_max = std::max(project_data.intensity_max, p.intensity);

        project_data.cave_aabb.min = glm::min(project_data.cave_aabb.min, p.position);
        project_data.cave_aabb.max = glm::max(project_data.cave_aabb.max, p.position);
    }

    bucketize_point_cloud(project_data.cave_vertices, project_data.buckets,
                          user_settings.io.map_load_extent,
                          user_settings.io.map_load_decimation_factor,
                          user_settings.io.map_load_decimation_levels,
                          user_settings.io.map_load_minimum_first_level_points,
                          user_settings.io.map_load_use_center_extent);

    for (auto& [ID, bucket] : project_data.buckets)
    {
        PointCloudLOD* current   = bucket.lods;
        int            lod_level = 0;
        while (current)
        {
            if (!current->points.empty())
            {
                current->vbo = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(current->points), current->points.data());
                current->vao = new VertexArray(current->vbo, false, nullptr, false, layout_point_intensity);

                spdlog::debug("Created LOD [{}] VAO [{}] and VBO [{}] for ID = [{} {} {}]", lod_level, current->vao->GetID(), current->vbo->GetID(), ID.x, ID.y, ID.z);
            }

            current = current->next;
            ++lod_level;
        }

        glm::vec3 min = bucket.aabb.min;
        glm::vec3 max = bucket.aabb.max;

        std::vector<Point> box_vertices = {
            {{min.x, min.y, min.z}},
            {{max.x, min.y, min.z}},
            {{max.x, min.y, min.z}},
            {{max.x, max.y, min.z}},
            {{max.x, max.y, min.z}},
            {{min.x, max.y, min.z}},
            {{min.x, max.y, min.z}},
            {{min.x, min.y, min.z}},

            {{min.x, min.y, max.z}},
            {{max.x, min.y, max.z}},
            {{max.x, min.y, max.z}},
            {{max.x, max.y, max.z}},
            {{max.x, max.y, max.z}},
            {{min.x, max.y, max.z}},
            {{min.x, max.y, max.z}},
            {{min.x, min.y, max.z}},

            {{min.x, min.y, min.z}},
            {{min.x, min.y, max.z}},
            {{max.x, min.y, min.z}},
            {{max.x, min.y, max.z}},
            {{max.x, max.y, min.z}},
            {{max.x, max.y, max.z}},
            {{min.x, max.y, min.z}},
            {{min.x, max.y, max.z}}};

        bucket.bbox_vbo = new Buffer(GL_NONE, std_vector_size(box_vertices), box_vertices.data());
        bucket.bbox_vao = new VertexArray(bucket.bbox_vbo, false, nullptr, false, layout_point); //
    }

    for (auto& [ID, bucket] : project_data.buckets)
    {
        size_t bucket_lod_count = 0;
        for (PointCloudLOD* lod = bucket.lods; lod; lod = lod->next)
        {
            ++bucket_lod_count;
        }

        project_data.max_lod_count = std::max(project_data.max_lod_count, bucket_lod_count);
    }

    if (project_data.buckets.empty())
    {
        spdlog::error("Bucketization produced no buckets from {} cave points", project_data.cave_vertices.size());
        return false;
    }

    return true;
}

// Load trajectory from explicit file path (returns true on success)
bool load_trajectory(ProjectData& project_data, const std::string& filename, const size_t trajectory_load_every_nth)
{
    if (!load_trajectory_csv(filename, project_data.trajectory_positions, project_data.trajectory_orientations_mat33, trajectory_load_every_nth))
    {
        spdlog::error("Failed to load trajectory CSV : {}", filename);
        return false;
    }

    if (!rebuild_trajectory_mat33_opengl_data(project_data))
    {
        spdlog::error("Failed to rebuild trajectory OpenGL data for : {}", filename);
        return false;
    }

    project_data.trajectory_path = filename;
    return true;
}

void load_trajectory_dialog(ProjectData& project_data, const UserSettings& user_settings)
{
    std::string filename;

    if (PFDOpenFile("Open CSV file", "CSV Files (.csv)", "*.csv", filename))
    {
        load_trajectory(project_data, filename, user_settings.io.trajectory_load_every_nth);
    }
}

// Load stretcher (object) from explicit file path (returns true on success)
bool load_object(ProjectData& project_data, const std::string& filename)
{
    if (!load_stretcher_ply(filename, project_data.stretcher_vertices, project_data.stretcher_indices))
    {
        spdlog::error("Failed to load stretcher PLY : {}", filename);
        return false;
    }

    if (!rebuild_stretcher_opengl_data(project_data))
    {
        spdlog::error("Failed to rebuild stretcher OpenGL data for : {}", filename);
        return false;
    }

    project_data.object_path = filename;
    return true;
}

void load_object_dialog(ProjectData& project_data)
{
    std::string filename;

    if (PFDOpenFile("Open PLY file", "PLY Files (.ply)", "*.ply", filename))
    {
        load_object(project_data, filename);
    }
}

// Load environment (cave point cloud) from explicit file path (returns true on success)
bool load_environment(ProjectData& project_data, const std::string& filename, const UserSettings& user_settings)
{
    if (!load_cave_laz(filename, project_data.cave_vertices))
    {
        spdlog::error("Failed to load environment LAZ : {}", filename);
        return false;
    }

    if (!rebuild_cave_opengl_data(project_data, user_settings))
    {
        spdlog::error("Failed to rebuild cave OpenGL data for : {}", filename);
        return false;
    }

    project_data.environment_path = filename;
    return true;
}

void load_environment_dialog(ProjectData& project_data, const UserSettings& user_settings)
{
    std::string filename;

    if (PFDOpenFile("Open LAZ file", "LAZ Files (*.laz *.las)", "*.laz *.las", filename))
    {
        load_environment(project_data, filename, user_settings);
    }
}

// Move trajectory index by +- given amount of meters along the trajectory (if possible)
void move_trajectory_index_by_distance(const std::vector<Point>& trajectory, uint32_t& index, const float amount)
{
    if (trajectory.empty())
    {
        spdlog::warn("Refusing to move trajectory index : no trajectory loaded");
        return;
    }

    const size_t last = trajectory.size() - 1;

    if (amount == 0.0f)
    {
        return;
    }

    if (index > last)
    {
        index = static_cast<uint32_t>(last);
    }

    const uint32_t previous_index = index;

    const glm::vec3 start_position = trajectory[index].position;

    float walked = 0.0f;

    if (amount >= 0.0f)
    {
        // Walk forward accumulating distance
        size_t i = index;
        while (i < last)
        {
            const float segment = glm::length(trajectory[i + 1].position - trajectory[i].position);
            walked += segment;
            ++i;

            if (walked >= amount)
            {
                break;
            }
        }

        if (walked < amount)
        {
            spdlog::debug("Requested {:.2f} m forward exceeds trajectory length : clamped to end", amount);
        }

        index = static_cast<uint32_t>(i);
    }
    else
    {
        // Walk backward accumulating distance
        const float target = -amount;
        size_t      i      = index;
        while (i > 0)
        {
            const float segment = glm::length(trajectory[i].position - trajectory[i - 1].position);
            walked += segment;
            --i;

            if (walked >= target)
            {
                break;
            }
        }

        if (walked < target)
        {
            spdlog::debug("Requested {:.2f} m backward exceeds trajectory length : clamped to start", target);
        }

        index = static_cast<uint32_t>(i);
    }

    spdlog::debug("Moved trajectory index from [{}] to [{}] by requested {:.2f} m (actual {:.2f} m)", previous_index, index, amount, glm::length(trajectory[index].position - start_position));
}
