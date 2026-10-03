#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include <Core/Camera.h>
#include <Core/PointCloud.h>
#include <Core/UserSettings.h>

#include <Core/OpenGL/Buffer.h>
#include <Core/OpenGL/VertexArray.h>

// A pair of 3-D world-space points picked by the user, with the computed distance
struct MeasurementEntry
{
    glm::vec3 point_a{};
    glm::vec3 point_b{};
    float     distance_m = 0.0f; // ||b - a|| in metres

    // Per-measurement colour (line + distance label)
    glm::vec3 color = glm::vec3(0.0f, 1.0f, 1.0f); // cyan by default

    // Per-measurement line width (1 to 8, default 2)
    float line_width = 2.0f;
};

// Transient picking state (first point waiting for second click) + list of completed measurements
struct MeasurementState
{
    // First point of the current in-progress measurement (nullopt = no pending point)
    std::optional<glm::vec3> pending_point{};

    // Completed measurement pairs
    std::vector<MeasurementEntry> entries{};
};

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
    Buffer*      trajectory_positions_vbo    = nullptr;
    VertexArray* trajectory_positions_vao    = nullptr;
    Buffer*      trajectory_orientations_vbo = nullptr; // vec3 columns (AxisX, AxisY, AxisZ) packed per vertex
    VertexArray* trajectory_axes_vao         = nullptr; // combined VAO: positions + orientation columns

    // TODO (m.wlasiuk) : limit based on point cloud statistics (8 * max points in LOD_0 accross PC ... ???)
    // GPU side data : collision points between stretcher OBB and point cloud (positions only, color as uniform)
    static constexpr size_t COLLISION_POINTS_CAPACITY = 1024 * 16;

    Buffer*      collision_points_vbo = nullptr;
    VertexArray* collision_points_vao = nullptr;

    // Trajectory playback state
    bool     trajectory_index_auto_play           = false;
    int32_t  trajectory_index_auto_play_increment = 1;
    uint32_t trajectory_index                     = 0;

    // Lock viewport 0 camera target to current trajectory pose
    bool lock_viewport0_target_to_trajectory = false;

    // Measurements : Shift+LMB precise point picks and computed distances
    MeasurementState measurements{};

    // GPU side data : measurement lines (2 vertices per completed entry + optional pending point marker)
    // Buffer holds ColoredVertex vertices; capacity is 2 * MEASUREMENT_LINE_CAPACITY entries
    static constexpr size_t MEASUREMENT_LINE_CAPACITY = 256;
    Buffer*                 measurement_line_vbo      = nullptr;
    VertexArray*            measurement_line_vao      = nullptr;

    // Multi-view rendering context : per-viewport cameras, camera modes, plane settings and window size
    MultiViewContext multi_view{};
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
