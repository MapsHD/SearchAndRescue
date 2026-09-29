#pragma once

#include <string>
#include <vector>

#include <cave-traversal-tool/PointCloud.h>
#include <cave-traversal-tool/UserSettings.h>

#include <cave-traversal-tool/OpenGL/Buffer.h>
#include <cave-traversal-tool/OpenGL/VertexArray.h>

// ProjectData : owns CPU and GPU side data for dataset and file paths
struct ProjectData
{
    // File paths
    std::string trajectory_path{};
    std::string object_path{};
    std::string environment_path{};

    // CPU side data : dataset
    std::vector<PointIntensity> cave_vertices{};
    PointCloudBucket            buckets{};
    size_t                      max_lod_count = 0;

    AABB  cave_aabb     = {};
    float intensity_min = 0.0f;
    float intensity_max = 1.0f;

    // CPU side data : stretcher (object)
    std::vector<ColorPoint> stretcher_vertices{};
    std::vector<uint32_t>   stretcher_indices{};
    AABB                    stretcher_aabb{};

    // CPU side data : trajectory
    std::vector<Point>                          trajectory_positions{};
    std::vector<TrajectoryPoseOrientationMat33> trajectory_orientations_mat33{};

    // GPU side data : stretcher (object)
    Buffer*      stretcher_vbo          = nullptr;
    Buffer*      stretcher_index_buffer = nullptr;
    VertexArray* stretcher_vao          = nullptr;

    // GPU side data : stretcher AABB
    Buffer*      stretcher_aabb_vbo = nullptr;
    VertexArray* stretcher_aabb_vao = nullptr;

    // GPU side data : trajectory
    Buffer*      trajectory_positions_vbo = nullptr;
    VertexArray* trajectory_positions_vao = nullptr;

    // TODO (m.wlasiuk) : limit based on point cloud statistics (8 * max points in LOD_0 accross PC ... ???)
    // GPU side data : collision points between stretcher OBB and point cloud (positions only, color as uniform)
    static constexpr size_t COLLISION_POINTS_CAPACITY = 1024 * 16;

    Buffer*      collision_points_vbo = nullptr;
    VertexArray* collision_points_vao = nullptr;

    // Level of detail : fixed LOD index vs automatic LOD from distance
    bool    use_fixed_lod   = true;
    int32_t fixed_lod_index = 0;

    // Trajectory playback state
    bool     trajectory_index_auto_play           = false;
    int32_t  trajectory_index_auto_play_increment = 1;
    uint32_t trajectory_index                     = 0;

    // Lock viewport 0 camera target to current trajectory pose
    bool lock_viewport0_target_to_trajectory = false;
};

// Size of vector contents in bytes (helper for OpenGL buffer uploads)
template <typename T>
size_t std_vector_size(const std::vector<T>& vector)
{
    return vector.size() * sizeof(T);
}

// Free all OpenGL resources owned by the project data
void free_project_data(ProjectData& project_data);

bool rebuild_trajectory_mat33_opengl_data(ProjectData& project_data);

bool rebuild_stretcher_opengl_data(ProjectData& project_data);

bool rebuild_cave_opengl_data(ProjectData& project_data, const UserSettings& user_settings);

bool load_trajectory(ProjectData& project_data, const std::string& filename, const size_t trajectory_load_every_nth = 1);

// Load trajectory from a file picked with a file dialog
void load_trajectory_dialog(ProjectData& project_data, const UserSettings& user_settings);

bool load_object(ProjectData& project_data, const std::string& filename);

// Load stretcher object from a file picked with a file dialog
void load_object_dialog(ProjectData& project_data);

bool load_environment(ProjectData& project_data, const std::string& filename, const UserSettings& user_settings);

// Load environment point cloud from a file picked with a file dialog
void load_environment_dialog(ProjectData& project_data, const UserSettings& user_settings);

// Move trajectory index by +- given amount of meters along the trajectory (if possible)
void move_trajectory_index_by_distance(const std::vector<Point>& trajectory, uint32_t& index, const float amount);
