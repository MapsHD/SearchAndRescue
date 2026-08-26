#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <ranges>
#include <sstream>
#include <vector>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// clang-format on

#include <cave-traversal-tool/ErrorCallbacks.h>
#include <cave-traversal-tool/FileIO.h>

#include <cave-traversal-tool/Camera.h>
#include <cave-traversal-tool/Config.h>
#include <cave-traversal-tool/Helpers.h>
#include <cave-traversal-tool/Structures.h>

#include <cave-traversal-tool/OpenGL/Buffer.h>
#include <cave-traversal-tool/OpenGL/Program.h>
#include <cave-traversal-tool/OpenGL/VertexArray.h>

#include <cave-traversal-tool/GPUData.h>
#include <cave-traversal-tool/Processing.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <ImGuizmo.h>

#include <portable-file-dialogs.h>

#include "from_hdmapping/pair_wise_iterative_closest_point.h"

#define CHECK_BOOL(expression)                                    \
    do                                                            \
    {                                                             \
        if (!expression)                                          \
        {                                                         \
            spdlog::error("Expression failed : {}", #expression); \
            std::abort();                                         \
        }                                                         \
    } while (0)

static std::vector<NormalPoint> g_cave_vertices{};
static PointCloudBucket         g_buckets{};

static uint32_t g_stretcher_aabb_vbo = UINT32_MAX;
static uint32_t g_stretcher_aabb_vao = UINT32_MAX;
static AABB     g_stretcher_aabb{};

static std::vector<ColorPoint>      g_stretcher_vertices{};
static std::vector<uint32_t>        g_stretcher_indices{};
static std::vector<Eigen::Vector3d> g_stretcher_icp_source{};

static std::vector<Point>                          g_trajectory_positions{};
static std::vector<TrajectoryPoseOrientationMat33> g_trajectory_orientations_mat33{};

static std::vector<Point>                          g_optimized_trajectory_positions{};
static std::vector<TrajectoryPoseOrientationMat33> g_optimized_trajectory_orientations_mat33{};

// static std::vector<Eigen::Affine3d> g_optimized_trajectory_poses{};

static float g_cpu_time_icp_ms                    = 0.0f;
static float g_cpu_time_draw_trajectory_ms        = 0.0f;
static float g_cpu_time_draw_stretcher_ms         = 0.0f;
static float g_cpu_time_draw_cave_buckets_ms      = 0.0f;
static float g_cpu_time_draw_cave_buckets_bbox_ms = 0.0f;

static float   g_cave_load_extent                     = 1.0f;
static int32_t g_cave_load_decimation_factor          = 2;
static int32_t g_cave_load_decimation_levels          = 8;
static int32_t g_cave_load_minimum_first_level_points = 250;
static bool    g_cave_load_use_centered_extents       = true;
static bool    g_cave_load_set_draw                   = true;
static int32_t g_load_csv_every_nth                   = 1;

static bool g_use_fine_picking = false;

static float g_cave_proximity_search = 3.0f;

static bool g_draw_origin               = true;
static bool g_draw_camera_target        = true;
static bool g_draw_trajectory           = true;
static bool g_draw_optimized_trajectory = true;
static bool g_draw_stretcher            = true;
static bool g_draw_stretcher_bbox       = true;
static bool g_draw_point_cloud          = true;
static bool g_draw_bounding_box         = true;

static bool      g_enable_inactive_state = false;
static glm::vec3 g_clear_color           = {0.2f, 0.2f, 0.2f};

static bool   g_enable_icp_optimization_for_autoplay = false;
static double g_icp_search_radious                   = 0.3;
static bool   g_icp_is_repulsion                     = true;
static int    g_icp_number_of_iterations             = 30;

static float g_origin_scale = 1.0f;
static float g_origin_width = 1.0f;

static float     g_target_scale = 1.0f;
static float     g_target_width = 1.0f;
static glm::vec3 g_target_color = {1.0f, 1.0f, 1.0f};

static float     g_trajectory_width = 1.0f;
static glm::vec3 g_trajectory_color = {1.0f, 1.0f, 1.0f};

static float     g_optimized_trajectory_width = 2.0f;
static glm::vec3 g_optimized_trajectory_color = {1.0f, 0.0f, 1.0f};

static glm::vec3 g_stretcher_box_color = {0.0f, 1.0f, 1.0f};
static float     g_stretcher_box_width = 1.0f;

static float g_point_cloud_point_size = 1.0f;

static float g_point_cloud_bbox_width                  = 1.0f;
static float g_point_cloud_bbox_in_obb_width           = 4.0f;
static float g_point_cloud_bbox_in_obb_proximity_width = 3.0f;

static bool g_point_cloud_bbox_draw                  = false;
static bool g_point_cloud_bbox_in_obb_draw           = true;
static bool g_point_cloud_bbox_in_obb_proximity_draw = true;

static bool g_point_cloud_bucket_draw                  = false;
static bool g_point_cloud_bucket_in_obb_draw           = true;
static bool g_point_cloud_bucket_in_obb_proximity_draw = true;

static uint32_t g_stretcher_vbo          = UINT32_MAX;
static uint32_t g_stretcher_index_buffer = UINT32_MAX;
static uint32_t g_stretcher_vao          = UINT32_MAX;

static uint32_t g_trajectory_positions_vbo = UINT32_MAX;
static uint32_t g_trajectory_positions_vao = UINT32_MAX;

static uint32_t g_optimized_trajectory_positions_vbo = UINT32_MAX;
static uint32_t g_optimized_trajectory_positions_vao = UINT32_MAX;

//
static bool     g_trajectory_index_auto_play           = false;
static int32_t  g_trajectory_index_auto_play_increment = 1;
static uint32_t g_trajectory_index                     = 0;

// gizmo
static bool g_modify_current_pose_with_gizmo = false;

glm::vec3 reorient(const glm::vec3& normal, const glm::vec3& normal_origin, const glm::vec3& point)
{
    return glm::dot(normal, point - normal_origin) >= 0.0f ? normal : -normal;
}

template <typename T>
static size_t std_vector_size(const std::vector<T>& vector)
{
    return vector.size() * sizeof(T);
}

static void rebuild_trajectory_mat33_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_point = opengl_vertex_array_get_vertex_layout<Point>();

    g_trajectory_index = 0;

    if (g_optimized_trajectory_positions_vao != UINT32_MAX)
    {
        spdlog::debug("Deleting old optimized trajectory positions VAO : {}", g_optimized_trajectory_positions_vao);
        glDeleteVertexArrays(1, &g_optimized_trajectory_positions_vao);
    }

    if (g_optimized_trajectory_positions_vbo != UINT32_MAX)
    {
        spdlog::debug("Deleting old optimized trajectory positions VBO : {}", g_optimized_trajectory_positions_vbo);
        glDeleteBuffers(1, &g_optimized_trajectory_positions_vbo);
    }

    if (g_trajectory_positions_vao != UINT32_MAX)
    {
        spdlog::debug("Deleting old trajectory positions VAO : {}", g_trajectory_positions_vao);
        glDeleteVertexArrays(1, &g_trajectory_positions_vao);
    }

    if (g_trajectory_positions_vbo != UINT32_MAX)
    {
        spdlog::debug("Deleting old trajectory positions VBO : {}", g_trajectory_positions_vbo);
        glDeleteBuffers(1, &g_trajectory_positions_vbo);
    }

    g_trajectory_positions_vbo = opengl_buffer_create_buffer(std_vector_size(g_trajectory_positions), GL_DYNAMIC_STORAGE_BIT, g_trajectory_positions.data());
    g_trajectory_positions_vao = opengl_vertex_array_create_vertex_array(g_trajectory_positions_vbo, true, UINT32_MAX, false, layout_point);

    g_optimized_trajectory_positions_vbo = opengl_buffer_create_buffer(std_vector_size(g_trajectory_positions), GL_DYNAMIC_STORAGE_BIT, nullptr);
    g_optimized_trajectory_positions_vao = opengl_vertex_array_create_vertex_array(g_optimized_trajectory_positions_vbo, true, UINT32_MAX, false, layout_point);

    spdlog::debug("Created VAO [{}] and VBO [{}]", g_trajectory_positions_vao, g_trajectory_positions_vbo);
}

static void rebuild_stretcher_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_color_point = opengl_vertex_array_get_vertex_layout<ColorPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point       = opengl_vertex_array_get_vertex_layout<Point>();

    if (g_stretcher_aabb_vao != UINT32_MAX)
    {
        spdlog::debug("Deleting old stretcher AABB positions VAO : {}", g_stretcher_aabb_vao);
        glDeleteVertexArrays(1, &g_stretcher_aabb_vao);
    }

    if (g_stretcher_aabb_vbo != UINT32_MAX)
    {
        spdlog::debug("Deleting old stretcher AABB positions VBO : {}", g_stretcher_aabb_vbo);
        glDeleteBuffers(1, &g_stretcher_aabb_vbo);
    }

    if (g_stretcher_vao != UINT32_MAX)
    {
        spdlog::debug("Deleting old stretcher positions VAO : {}", g_stretcher_vao);
        glDeleteVertexArrays(1, &g_stretcher_vao);
    }

    if (g_stretcher_vbo != UINT32_MAX)
    {
        spdlog::debug("Deleting old stretcher positions VBO : {}", g_stretcher_vbo);
        glDeleteBuffers(1, &g_stretcher_vbo);
    }

    if (g_stretcher_index_buffer != UINT32_MAX)
    {
        spdlog::debug("Deleting old stretcher index IBO : {}", g_stretcher_index_buffer);
        glDeleteBuffers(1, &g_stretcher_index_buffer);
    }

    g_stretcher_aabb.min = g_stretcher_vertices[0].position;
    g_stretcher_aabb.max = g_stretcher_vertices[0].position;

    for (const auto& v : g_stretcher_vertices)
    {
        const glm::vec3& p = v.position;

        g_stretcher_aabb.min = glm::min(g_stretcher_aabb.min, p);
        g_stretcher_aabb.max = glm::max(g_stretcher_aabb.max, p);
    }

    std::vector<Point> line_vertices{
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.min.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.min.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.min.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.max.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.max.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.max.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.max.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.min.y, g_stretcher_aabb.min.z}},

        {{g_stretcher_aabb.min.x, g_stretcher_aabb.min.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.min.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.min.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.max.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.max.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.max.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.max.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.min.y, g_stretcher_aabb.max.z}},

        {{g_stretcher_aabb.min.x, g_stretcher_aabb.min.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.min.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.min.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.min.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.max.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.max.x, g_stretcher_aabb.max.y, g_stretcher_aabb.max.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.max.y, g_stretcher_aabb.min.z}},
        {{g_stretcher_aabb.min.x, g_stretcher_aabb.max.y, g_stretcher_aabb.max.z}}};

    g_stretcher_aabb_vbo = opengl_buffer_create_buffer(std_vector_size(line_vertices), GL_NONE, line_vertices.data());
    g_stretcher_aabb_vao = opengl_vertex_array_create_vertex_array(g_stretcher_aabb_vbo, true, UINT32_MAX, 0, layout_point);

    g_stretcher_vbo          = opengl_buffer_create_buffer(std_vector_size(g_stretcher_vertices), GL_DYNAMIC_STORAGE_BIT, g_stretcher_vertices.data());
    g_stretcher_index_buffer = opengl_buffer_create_buffer(std_vector_size(g_stretcher_indices), GL_DYNAMIC_STORAGE_BIT, g_stretcher_indices.data());

    g_stretcher_vao = opengl_vertex_array_create_vertex_array(g_stretcher_vbo, true, g_stretcher_index_buffer, true, layout_color_point);

    spdlog::debug("Created VAO [{}], VBO [{}] and IBO [{}]", g_stretcher_vao, g_stretcher_vbo, g_stretcher_index_buffer);
}

static void rebuild_cave_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_normal_point = opengl_vertex_array_get_vertex_layout<NormalPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point        = opengl_vertex_array_get_vertex_layout<Point>();

    // Clear old data
    for (auto& [ID, bucket] : g_buckets)
    {
        // Clear LODs
        PointCloudLOD* current = bucket.lods;
        while (current)
        {
            if (current->vao != UINT32_MAX)
            {
                spdlog::debug("Deleting old cave [ID = {} {} {}] VAO [{}]", ID.x, ID.y, ID.z, current->vao);
                glDeleteVertexArrays(1, &current->vao);
                current->vao = UINT32_MAX;
            }

            if (current->vbo != UINT32_MAX)
            {
                spdlog::debug("Deleting old cave [ID = {} {} {}] VBO [{}]", ID.x, ID.y, ID.z, current->vbo);
                glDeleteBuffers(1, &current->vbo);
                current->vbo = UINT32_MAX;
            }

            current->points.clear();
            PointCloudLOD* next = current->next;
            delete current;
            current = next;
        }
        bucket.lods = nullptr;
        bucket.draw = false;

        if (bucket.bbox_vao != UINT32_MAX)
        {
            spdlog::debug("Deleting old bounding box VAO [{}] for ID = {} {} {}", bucket.bbox_vao, ID.x, ID.y, ID.z);
            glDeleteVertexArrays(1, &bucket.bbox_vao);
            bucket.bbox_vao = UINT32_MAX;
        }

        if (bucket.bbox_vbo != UINT32_MAX)
        {
            spdlog::debug("Deleting old bounding box VBO [{}] for ID = {} {} {}", bucket.bbox_vbo, ID.x, ID.y, ID.z);
            glDeleteBuffers(1, &bucket.bbox_vbo);
            bucket.bbox_vbo = UINT32_MAX;
        }
    }

    g_buckets.clear();

    bucketize_point_cloud(g_cave_vertices, g_buckets, g_cave_load_extent, g_cave_load_decimation_factor, g_cave_load_decimation_levels, g_cave_load_minimum_first_level_points, g_cave_load_use_centered_extents, g_cave_load_use_centered_extents);

    for (auto& [ID, bucket] : g_buckets)
    {
        PointCloudLOD* current   = bucket.lods;
        int            lod_level = 0;
        while (current)
        {
            if (!current->points.empty())
            {
                current->vbo = opengl_buffer_create_buffer(std_vector_size(current->points), GL_DYNAMIC_STORAGE_BIT, current->points.data());
                current->vao = opengl_vertex_array_create_vertex_array(current->vbo, true, UINT32_MAX, 0, layout_normal_point);

                spdlog::debug("Created LOD [{}] VAO [{}] and VBO [{}] for ID = [{} {} {}]", lod_level, current->vao, current->vbo, ID.x, ID.y, ID.z);
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

        bucket.bbox_vbo = opengl_buffer_create_buffer(std_vector_size(box_vertices), GL_NONE, box_vertices.data());
        bucket.bbox_vao = opengl_vertex_array_create_vertex_array(bucket.bbox_vbo, true, UINT32_MAX, false, layout_point);
    }
}

static inline void load_trajectory()
{
    const std::vector<std::string> result = pfd::open_file(
                                                "Open CSV file",
                                                "",
                                                {"CSV Files (.csv)", "*.csv", "All Files", "*"})
                                                .result();

    if (result.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    if (result.size() > 1)
    {
        spdlog::info("PFD result contains multiple entiries - loading first ...");
    }

    const std::string& filename = result.front();

    CHECK_BOOL(load_trajectory_csv_mat33(filename, g_trajectory_positions, g_trajectory_orientations_mat33, g_load_csv_every_nth));
    rebuild_trajectory_mat33_opengl_data();
}

static inline void load_trajectory_bin()
{
    const std::vector<std::string> result = pfd::open_file(
                                                "Load trajectory (MAT33) - BINARY",
                                                "",
                                                {"Binary files", "*.bin"})
                                                .result();

    if (result.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    if (result.size() > 1)
    {
        spdlog::info("PFD result contains multiple entiries - loading first ...");
    }

    const std::string& filename = result.front();

    CHECK_BOOL(load_trajectory_bin_mat33(filename, g_trajectory_positions, g_trajectory_orientations_mat33, g_load_csv_every_nth));
    rebuild_trajectory_mat33_opengl_data();
}

static inline void save_trajectory_bin()
{
    std::string filename = pfd::save_file(
                               "Save trajectory (MAT33) - BINARY",
                               "",
                               {"Binary files", "*.bin"})
                               .result();

    if (filename.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    CHECK_BOOL(save_trajectory_bin_mat33(filename, g_trajectory_positions, g_trajectory_orientations_mat33));
}

static inline void load_object()
{
    const std::vector<std::string> result = pfd::open_file(
                                                "Open PLY file",
                                                "",
                                                {"PLY Files (.ply)", "*.ply", "All Files", "*"})
                                                .result();

    if (result.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    if (result.size() > 1)
    {
        spdlog::info("PFD result contains multiple entiries - loading first ...");
    }

    const std::string& filename = result.front();

    CHECK_BOOL(load_stretcher_ply(filename, g_stretcher_vertices, g_stretcher_indices));
    rebuild_stretcher_opengl_data();

    g_stretcher_icp_source = {};

    for (const auto& stretcher_vertex : g_stretcher_vertices)
    {
        g_stretcher_icp_source.push_back(Eigen::Vector3d(stretcher_vertex.position.x, stretcher_vertex.position.y, stretcher_vertex.position.z));
    }
}

static inline void load_object_bin()
{
    const std::vector<std::string> result = pfd::open_file(
                                                "Open PLY file - BIN",
                                                "",
                                                {"PLY Files (.bin)", "*.bin"})
                                                .result();

    if (result.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    if (result.size() > 1)
    {
        spdlog::info("PFD result contains multiple entiries - loading first ...");
    }

    const std::string& filename = result.front();

    CHECK_BOOL(load_stretcher_bin(filename, g_stretcher_vertices, g_stretcher_indices));
    rebuild_stretcher_opengl_data();

    g_stretcher_icp_source = {};

    for (const auto& stretcher_vertex : g_stretcher_vertices)
    {
        g_stretcher_icp_source.push_back(Eigen::Vector3d(stretcher_vertex.position.x, stretcher_vertex.position.y, stretcher_vertex.position.z));
    }
}

static inline void save_object_bin()
{
    std::string filename = pfd::save_file(
                               "Save object - BINARY",
                               "",
                               {"Binary files", "*.bin"})
                               .result();

    if (filename.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    CHECK_BOOL(save_stretcher_bin(filename, g_stretcher_vertices, g_stretcher_indices));
}

static inline void load_environment()
{
    const std::vector<std::string> result = pfd::open_file(
                                                "Open PLY file",
                                                "",
                                                {"PLY Files (.ply)", "*.ply", "All Files", "*"})
                                                .result();

    if (result.empty())
    {
        spdlog::info("PFD result empty - doing nothing ...");
        return;
    }

    if (result.size() > 1)
    {
        spdlog::info("PFD result contains multiple entries - loading first ...");
    }

    const std::string& filename = result.front();

    CHECK_BOOL(load_cave_ply(filename, g_cave_vertices));
    rebuild_cave_opengl_data();
}

static inline void load_environment_bin()
{
    const std::vector<std::string> result = pfd::open_file(
                                                "Open environment PLY file - BIN",
                                                "",
                                                {"PLY Files (.bin)", "*.bin"})
                                                .result();

    if (result.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    if (result.size() > 1)
    {
        spdlog::info("PFD result contains multiple entiries - loading first ...");
    }

    const std::string& filename = result.front();

    CHECK_BOOL(load_cave_bin(filename, g_cave_vertices));
    rebuild_cave_opengl_data();
}

static inline void save_environment_bin()
{
    std::string filename = pfd::save_file(
                               "Save environment - BINARY",
                               "",
                               {"Binary files", "*.bin"})
                               .result();

    if (filename.empty())
    {
        spdlog::info("PFD result emplty - doing nothing ...");
        return;
    }

    CHECK_BOOL(save_cave_bin(filename, g_cave_vertices));
}

int main()
{
    std::vector<ColorPoint> origin = {
        {{0.0f, 0.0f, 0.0f}, {0xFF, 0x00, 0x00}},
        {{1.0f, 0.0f, 0.0f}, {0xFF, 0x00, 0x00}},
        {{0.0f, 0.0f, 0.0f}, {0x00, 0xFF, 0x00}},
        {{0.0f, 1.0f, 0.0f}, {0x00, 0xFF, 0x00}},
        {{0.0f, 0.0f, 0.0f}, {0x00, 0x00, 0xFF}},
        {{0.0f, 0.0f, 1.0f}, {0x00, 0x00, 0xFF}}};

    std::vector<Point> target = {
        {{-1.0f, 0.0f, 0.0f}},
        {{1.0f, 0.0f, 0.0f}},
        {{0.0f, -1.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, -1.0f}},
        {{0.0f, 0.0f, 1.0f}}};

    std::vector<char> origin_vertex_shader_source   = {};
    std::vector<char> origin_fragment_shader_source = {};

    std::vector<char> camera_target_vertex_shader_source   = {};
    std::vector<char> camera_target_fragment_shader_source = {};

    std::vector<char> point_cloud_vertex_shader_source   = {};
    std::vector<char> point_cloud_fragment_shader_source = {};

    std::vector<char> trajectory_vertex_shader_source   = {};
    std::vector<char> trajectory_fragment_shader_source = {};

    std::vector<char> stretcher_vertex_shader_source   = {};
    std::vector<char> stretcher_fragment_shader_source = {};

    std::vector<char> bounding_box_vertex_shader_source   = {};
    std::vector<char> bounding_box_fragment_shader_source = {};

    std::vector<char> bounding_box_stretcher_vertex_shader_source   = {};
    std::vector<char> bounding_box_stretcher_fragment_shader_source = {};

    CHECK_BOOL(load_text_file(origin_vertex_shader_source, std::filesystem::path(SHADERS_DIR).append("origin.vert")));
    CHECK_BOOL(load_text_file(origin_fragment_shader_source, std::filesystem::path(SHADERS_DIR).append("origin.frag")));

    CHECK_BOOL(load_text_file(camera_target_vertex_shader_source, std::filesystem::path(SHADERS_DIR).append("camera_target.vert")));
    CHECK_BOOL(load_text_file(camera_target_fragment_shader_source, std::filesystem::path(SHADERS_DIR).append("camera_target.frag")));

    CHECK_BOOL(load_text_file(point_cloud_vertex_shader_source, std::filesystem::path(SHADERS_DIR).append("point_cloud.vert")));
    CHECK_BOOL(load_text_file(point_cloud_fragment_shader_source, std::filesystem::path(SHADERS_DIR).append("point_cloud.frag")));

    CHECK_BOOL(load_text_file(trajectory_vertex_shader_source, std::filesystem::path(SHADERS_DIR).append("trajectory.vert")));
    CHECK_BOOL(load_text_file(trajectory_fragment_shader_source, std::filesystem::path(SHADERS_DIR).append("trajectory.frag")));

    CHECK_BOOL(load_text_file(stretcher_vertex_shader_source, std::filesystem::path(SHADERS_DIR).append("stretcher.vert")));
    CHECK_BOOL(load_text_file(stretcher_fragment_shader_source, std::filesystem::path(SHADERS_DIR).append("stretcher.frag")));

    CHECK_BOOL(load_text_file(bounding_box_vertex_shader_source, std::filesystem::path(SHADERS_DIR).append("bounding_box.vert")));
    CHECK_BOOL(load_text_file(bounding_box_fragment_shader_source, std::filesystem::path(SHADERS_DIR).append("bounding_box.frag")));

    CHECK_BOOL(load_text_file(bounding_box_stretcher_vertex_shader_source, std::filesystem::path(SHADERS_DIR).append("bounding_box_stretcher.vert")));
    CHECK_BOOL(load_text_file(bounding_box_stretcher_fragment_shader_source, std::filesystem::path(SHADERS_DIR).append("bounding_box_stretcher.frag")));

    glfwSetErrorCallback(ErrorCallback::GLFW);
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);

    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_CONTEXT_NO_ERROR, GLFW_FALSE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "cave-traversal-tool-application", nullptr, nullptr);

    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetWindowSizeCallback(window, size_callback);

    Camera camera{};
    glfwSetWindowUserPointer(window, &camera);

    glfwMakeContextCurrent(window);

    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    // glEnable(GL_DEBUG_OUTPUT);
    // glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    // glDebugMessageCallback(ErrorCallback::OpenGL, nullptr);

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    uint32_t origin_vertex_shader_id   = opengl_program_compile_shader(origin_vertex_shader_source, GL_VERTEX_SHADER);
    uint32_t origin_fragment_shader_id = opengl_program_compile_shader(origin_fragment_shader_source, GL_FRAGMENT_SHADER);
    uint32_t origin_program            = opengl_program_create_program(origin_vertex_shader_id, origin_fragment_shader_id);

    uint32_t camera_target_vertex_shader_id   = opengl_program_compile_shader(camera_target_vertex_shader_source, GL_VERTEX_SHADER);
    uint32_t camera_target_fragment_shader_id = opengl_program_compile_shader(camera_target_fragment_shader_source, GL_FRAGMENT_SHADER);
    uint32_t camera_target_program            = opengl_program_create_program(camera_target_vertex_shader_id, camera_target_fragment_shader_id);

    uint32_t point_cloud_vertex_shader_id   = opengl_program_compile_shader(point_cloud_vertex_shader_source, GL_VERTEX_SHADER);
    uint32_t point_cloud_fragment_shader_id = opengl_program_compile_shader(point_cloud_fragment_shader_source, GL_FRAGMENT_SHADER);
    uint32_t point_cloud_program            = opengl_program_create_program(point_cloud_vertex_shader_id, point_cloud_fragment_shader_id);

    uint32_t trajectory_vertex_shader_id   = opengl_program_compile_shader(trajectory_vertex_shader_source, GL_VERTEX_SHADER);
    uint32_t trajectory_fragment_shader_id = opengl_program_compile_shader(trajectory_fragment_shader_source, GL_FRAGMENT_SHADER);
    uint32_t trajectory_program            = opengl_program_create_program(trajectory_vertex_shader_id, trajectory_fragment_shader_id);

    uint32_t stretcher_vertex_shader_id   = opengl_program_compile_shader(stretcher_vertex_shader_source, GL_VERTEX_SHADER);
    uint32_t stretcher_fragment_shader_id = opengl_program_compile_shader(stretcher_fragment_shader_source, GL_FRAGMENT_SHADER);
    uint32_t stretcher_program            = opengl_program_create_program(stretcher_vertex_shader_id, stretcher_fragment_shader_id);

    uint32_t bounding_box_vertex_shader_id   = opengl_program_compile_shader(bounding_box_vertex_shader_source, GL_VERTEX_SHADER);
    uint32_t bounding_box_fragment_shader_id = opengl_program_compile_shader(bounding_box_fragment_shader_source, GL_FRAGMENT_SHADER);
    uint32_t bounding_box_program            = opengl_program_create_program(bounding_box_vertex_shader_id, bounding_box_fragment_shader_id);

    uint32_t bounding_box_stretcher_vertex_shader_id   = opengl_program_compile_shader(bounding_box_stretcher_vertex_shader_source, GL_VERTEX_SHADER);
    uint32_t bounding_box_stretcher_fragment_shader_id = opengl_program_compile_shader(bounding_box_stretcher_fragment_shader_source, GL_FRAGMENT_SHADER);
    uint32_t bounding_box_stretcher_program            = opengl_program_create_program(bounding_box_stretcher_vertex_shader_id, bounding_box_stretcher_fragment_shader_id);

    const std::vector<VertexBufferAttributeLayout> layout_color_point = opengl_vertex_array_get_vertex_layout<ColorPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point       = opengl_vertex_array_get_vertex_layout<Point>();

    uint32_t origin_buffer = opengl_buffer_create_buffer(std_vector_size(origin), GL_DYNAMIC_STORAGE_BIT, origin.data());
    uint32_t origin_vao    = opengl_vertex_array_create_vertex_array(origin_buffer, true, UINT32_MAX, false, layout_color_point);

    uint32_t target_buffer = opengl_buffer_create_buffer(std_vector_size(target), GL_DYNAMIC_STORAGE_BIT, target.data());
    uint32_t target_vao    = opengl_vertex_array_create_vertex_array(target_buffer, true, UINT32_MAX, false, layout_point);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        if (g_enable_inactive_state && (!glfwGetWindowAttrib(window, GLFW_FOCUSED) || glfwGetWindowAttrib(window, GLFW_ICONIFIED)))
        {
            glfwSetWindowTitle(window, "cave-traversal-tool-application [INACTIVE]");

            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }
        else
        {
            glfwSetWindowTitle(window, "cave-traversal-tool-application [ACTIVE]");
        }

        int32_t width  = 0;
        int32_t height = 0;
        glfwGetWindowSize(window, &width, &height);

        glViewport(0, 0, width, height);

        glClearColor(g_clear_color.x, g_clear_color.y, g_clear_color.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspectiveFov(glm::radians(55.0f), static_cast<float>(width), static_cast<float>(height), 0.1f, 1000.0f);
        glm::mat4 view       = camera.get_view();
        glm::mat4 MVP        = projection * view;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        float line_width_range[2] = {1.0f, 1.0f};
        glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, line_width_range);

        float point_size_range[2] = {1.0f, 1.0f};
        glGetFloatv(GL_POINT_SIZE_RANGE, point_size_range);

        const float line_width_min = line_width_range[0];
        const float line_width_max = line_width_range[1];

        const float point_size_min = point_size_range[0];
        const float point_size_max = point_size_range[1];

        const bool can_draw_trajectory           = g_trajectory_positions.size() && g_trajectory_orientations_mat33.size();
        const bool can_draw_optimized_trajectory = g_optimized_trajectory_positions.size() && g_optimized_trajectory_orientations_mat33.size();
        const bool can_draw_stretcher            = g_stretcher_vertices.size() && g_stretcher_indices.size();
        const bool can_draw_cave                 = g_buckets.size();
        const bool can_draw_bounding_boxes       = g_buckets.size();

        std::array<glm::vec4, 6> frustum{};
        compute_camera_frustum_planes(view, projection, frustum);

        glm::vec3 stretcher_position    = glm::vec3(0.0f);
        glm::mat3 stretcher_orientation = glm::mat3(1.0f);

        if (g_trajectory_index_auto_play && !g_trajectory_orientations_mat33.empty())
        {
            const size_t last = g_trajectory_orientations_mat33.size() - 1;

            if (g_trajectory_index + g_trajectory_index_auto_play_increment >= last)
            {
                g_trajectory_index           = last;
                g_trajectory_index_auto_play = false;
            }
            else
            {
                g_trajectory_index += g_trajectory_index_auto_play_increment;
            }
        }

        if (g_trajectory_positions.size() && g_trajectory_orientations_mat33.size())
        {
            const auto& trajectory_point      = g_trajectory_positions[g_trajectory_index];
            const auto& trajectoryorientation = g_trajectory_orientations_mat33[g_trajectory_index];

            stretcher_position    = trajectory_point.position;
            stretcher_orientation = trajectoryorientation.orientation;
        }

        glm::mat4 stretcher_pose = glm::translate(glm::mat4(1.0f), stretcher_position) * glm::mat4(stretcher_orientation);

        const OBB  stretcher_obb               = aabb_to_obb(g_stretcher_aabb, stretcher_pose);
        const auto in_obb_ids_in_obb_proximity = find_buckets_in_obb(g_buckets, stretcher_obb, g_cave_proximity_search);

        if (ImGui::Begin("application"))
        {
            ImGui::Separator();
            if (ImGui::TreeNode("Performance"))
            {
                ImGui::Text("Framerate  : %.3f FPS", ImGui::GetIO().Framerate);
                ImGui::Text("Frame time : %.3f ms", ImGui::GetIO().DeltaTime * 1000.0f);
                ImGui::Text("g_cpu_time_icp_ms                    : %.3f ms", g_cpu_time_icp_ms);
                ImGui::Text("g_cpu_time_draw_trajectory_ms        : %.3f ms", g_cpu_time_draw_trajectory_ms);
                ImGui::Text("g_cpu_time_draw_stretcher_ms         : %.3f ms", g_cpu_time_draw_stretcher_ms);
                ImGui::Text("g_cpu_time_draw_cave_buckets_ms      : %.3f ms", g_cpu_time_draw_cave_buckets_ms);
                ImGui::Text("g_cpu_time_draw_cave_buckets_bbox_ms : %.3f ms", g_cpu_time_draw_cave_buckets_bbox_ms);
                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("File input / output"))
            {
                if (ImGui::BeginTable("IOGrid", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchSame))
                {
                    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                    ImGui::TableSetupColumn("Read", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Read Binary", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Save Binary", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("LAS/LAZ", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableHeadersRow();

                    // Trajectory
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Trajectory");

                    ImGui::TableSetColumnIndex(1);
                    if (ImGui::Button("##traj_read", ImVec2(-FLT_MIN, 0)))
                    {
                        load_trajectory();
                    }

                    ImGui::TableSetColumnIndex(2);
                    if (ImGui::Button("##traj_read_bin", ImVec2(-FLT_MIN, 0)))
                    {
                        load_trajectory_bin();
                    }

                    ImGui::TableSetColumnIndex(3);
                    if (ImGui::Button("##traj_save_bin", ImVec2(-FLT_MIN, 0)))
                    {
                        save_trajectory_bin();
                    }

                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("");

                    // Object
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Object");

                    ImGui::TableSetColumnIndex(1);
                    if (ImGui::Button("##obj_read", ImVec2(-FLT_MIN, 0)))
                    {
                        load_object();
                    }

                    ImGui::TableSetColumnIndex(2);
                    if (ImGui::Button("##obj_read_bin", ImVec2(-FLT_MIN, 0)))
                    {
                        load_object_bin();
                    }

                    ImGui::TableSetColumnIndex(3);
                    if (ImGui::Button("##obj_save_bin", ImVec2(-FLT_MIN, 0)))
                    {
                        save_object_bin();
                    }

                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("");

                    // Environment
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Environment");

                    ImGui::TableSetColumnIndex(1);
                    if (ImGui::Button("##env_read", ImVec2(-FLT_MIN, 0)))
                    {
                        load_environment();
                    }

                    ImGui::TableSetColumnIndex(2);
                    if (ImGui::Button("##env_read_bin", ImVec2(-FLT_MIN, 0)))
                    {
                        load_environment_bin();
                    }

                    ImGui::TableSetColumnIndex(3);
                    if (ImGui::Button("##env_save_bin", ImVec2(-FLT_MIN, 0)))
                    {
                        save_environment_bin();
                    }

                    ImGui::TableSetColumnIndex(4);

                    ImGui::EndTable();
                }

                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("File input / output options"))
            {
                ImGui::DragFloat("g_cave_load_extent", &g_cave_load_extent, 0.01f, 0.1f, FLT_MAX, "%.3f");
                ImGui::DragInt("g_cave_load_decimation_factor", &g_cave_load_decimation_factor, 1.0f, 1, INT32_MAX);
                ImGui::DragInt("g_cave_load_decimation_levels", &g_cave_load_decimation_levels, 1.0f, 1, INT32_MAX);
                ImGui::DragInt("g_cave_load_minimum_first_level_points", &g_cave_load_minimum_first_level_points, 1.0f, 1, INT32_MAX);
                ImGui::Checkbox("g_cave_load_use_centered_extents", &g_cave_load_use_centered_extents);
                ImGui::Checkbox("g_cave_load_set_draw", &g_cave_load_set_draw);
                ImGui::DragInt("g_load_csv_every_nth", &g_load_csv_every_nth, 1.0f, 1, INT32_MAX);
                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("Picking options"))
            {
                ImGui::Checkbox("g_use_fine_picking", &g_use_fine_picking);
                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("Proximity search"))
            {
                ImGui::DragFloat("g_cave_proximity_search", &g_cave_proximity_search, 0.1f, 0.0f, FLT_MAX);
                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("Can draw data?"))
            {
                const ImVec4 green(0.2f, 1.0f, 0.2f, 1.0f);
                const ImVec4 orange(1.0f, 0.6f, 0.0f, 1.0f);

                ImGui::Text("can_draw_trajectory : ");
                ImGui::SameLine();
                ImGui::TextColored(can_draw_trajectory ? green : orange, "%c", can_draw_trajectory ? 'Y' : 'N');

                ImGui::Text("can_draw_stretcher : ");
                ImGui::SameLine();
                ImGui::TextColored(can_draw_stretcher ? green : orange, "%c", can_draw_stretcher ? 'Y' : 'N');

                ImGui::Text("can_draw_cave : ");
                ImGui::SameLine();
                ImGui::TextColored(can_draw_cave ? green : orange, "%c", can_draw_cave ? 'Y' : 'N');

                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("Draw enable options"))
            {
                ImGui::Checkbox("g_draw_origin", &g_draw_origin);
                ImGui::Checkbox("g_draw_camera_target", &g_draw_camera_target);
                ImGui::Checkbox("g_draw_trajectory", &g_draw_trajectory);
                ImGui::Checkbox("g_draw_optimized_trajectory", &g_draw_optimized_trajectory);
                ImGui::Checkbox("g_draw_stretcher", &g_draw_stretcher);
                ImGui::Checkbox("g_draw_stretcher_bbox", &g_draw_stretcher_bbox);
                ImGui::Checkbox("g_draw_point_cloud", &g_draw_point_cloud);
                ImGui::Checkbox("g_draw_bounding_box", &g_draw_bounding_box);

                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("OpenGL"))
            {
                ImGui::Text("OpenGL version      : %s", (const char*)glGetString(GL_VERSION));
                ImGui::Text("OpenGL vendor       : %s", (const char*)glGetString(GL_VENDOR));
                ImGui::Text("OpenGL renderer     : %s", (const char*)glGetString(GL_RENDERER));
                ImGui::Text("OpenGL GLSL version : %s", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

                ImGui::Checkbox("g_enable_inactive_state", &g_enable_inactive_state);
                ImGui::ColorEdit3("g_clear_color", glm::value_ptr(g_clear_color));
                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("ICP"))
            {
                const double search_radious_min = 0.0;
                const double search_radious_max = DBL_MAX;

                ImGui::Checkbox("g_enable_icp_optimization_for_autoplay", &g_enable_icp_optimization_for_autoplay);
                ImGui::DragScalar("g_icp_search_radious", ImGuiDataType_Double, &g_icp_search_radious, 0.01f, &search_radious_min, &search_radious_max);
                ImGui::Checkbox("g_icp_is_repulsion", &g_icp_is_repulsion);
                ImGui::DragInt("g_icp_number_of_iterations", &g_icp_number_of_iterations, 1.0f, 1, INT32_MAX);

                if (ImGui::Button("Save resulting optimized trajectory"))
                {
                    std::string filename = pfd::save_file(
                                               "Save trajectory result - TXT",
                                               "",
                                               {"Text files", "*.txt"})
                                               .result();

                    if (filename.empty())
                    {
                        spdlog::info("PFD result emplty - doing nothing ...");
                    }
                    else
                    {
                        __debugbreak();
                        // write_affine3d_trajectory_to_txt(g_optimized_trajectory_poses, filename);
                    }
                }

                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("Display"))
            {
                ImGui::DragFloat("g_origin_scale", &g_origin_scale, 0.1f, 1.0f, FLT_MAX);
                ImGui::DragFloat("g_origin_width", &g_origin_width, 1.0f, line_width_min, line_width_max);

                ImGui::Separator();
                ImGui::DragFloat("g_target_scale", &g_target_scale, 0.1f, 1.0f, FLT_MAX);
                ImGui::DragFloat("g_target_width", &g_target_width, 1.0f, line_width_min, line_width_max);
                ImGui::ColorEdit3("g_target_color", glm::value_ptr(g_target_color));

                ImGui::Separator();
                ImGui::DragFloat("g_trajectory_width", &g_trajectory_width, 1.0f, line_width_min, line_width_max);
                ImGui::ColorEdit3("g_trajectory_color", glm::value_ptr(g_trajectory_color));

                ImGui::Separator();
                ImGui::DragFloat("g_optimized_trajectory_width", &g_optimized_trajectory_width, 1.0f, line_width_min, line_width_max);
                ImGui::ColorEdit3("g_optimized_trajectory_color", glm::value_ptr(g_optimized_trajectory_color));

                ImGui::Separator();
                ImGui::ColorEdit3("g_stretcher_box_color", glm::value_ptr(g_stretcher_box_color));
                ImGui::DragFloat("g_stretcher_box_width", &g_stretcher_box_width, 1.0f, line_width_min, line_width_max);

                ImGui::Separator();
                ImGui::DragFloat("g_point_cloud_point_size", &g_point_cloud_point_size, 1.0f, point_size_min, point_size_max);

                ImGui::Separator();
                ImGui::DragFloat("g_point_cloud_bbox_width", &g_point_cloud_bbox_width, 1.0f, line_width_min, line_width_max);
                ImGui::DragFloat("g_point_cloud_bbox_in_obb_width", &g_point_cloud_bbox_in_obb_width, 1.0f, line_width_min, line_width_max);
                ImGui::DragFloat("g_point_cloud_bbox_in_obb_proximity_width", &g_point_cloud_bbox_in_obb_proximity_width, 1.0f, line_width_min, line_width_max);

                ImGui::Separator();
                ImGui::Checkbox("g_point_cloud_bbox_draw", &g_point_cloud_bbox_draw);
                ImGui::Checkbox("g_point_cloud_bbox_in_obb_draw", &g_point_cloud_bbox_in_obb_draw);
                ImGui::Checkbox("g_point_cloud_bbox_in_obb_proximity_draw", &g_point_cloud_bbox_in_obb_proximity_draw);

                ImGui::Separator();
                ImGui::Checkbox("g_point_cloud_bucket_draw", &g_point_cloud_bucket_draw);
                ImGui::Checkbox("g_point_cloud_bucket_in_obb_draw", &g_point_cloud_bucket_in_obb_draw);
                ImGui::Checkbox("g_point_cloud_bucket_in_obb_proximity_draw", &g_point_cloud_bucket_in_obb_proximity_draw);

                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("Trrajectory"))
            {
                if (g_trajectory_orientations_mat33.size())
                {
                    const uint32_t zero                  = 0U;
                    const uint32_t max_orientation_index = static_cast<uint32_t>(g_trajectory_orientations_mat33.size()) - 1U;

                    ImGui::Text("Trajectory : %zu / %zu = %.2f%", static_cast<size_t>(g_trajectory_index), max_orientation_index, static_cast<float>(g_trajectory_index) / static_cast<float>(max_orientation_index) * 100.0f);
                    ImGui::Checkbox("g_trajectory_index_auto_play", &g_trajectory_index_auto_play);
                    ImGui::DragInt("g_trajectory_index_auto_play_increment", &g_trajectory_index_auto_play_increment, 1.0f, 1, INT32_MAX);
                    ImGui::DragScalar("g_trajectory_index", ImGuiDataType_U32, &g_trajectory_index, 1.0f, &zero, &max_orientation_index);
                }
                else
                {
                    ImGui::TextColored({1.0f, 0.0f, 0.0f, 1.0f}, "Can not set index of trajectory pose - load trajectory!");
                }

                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("Buckets"))
            {
                if (ImGui::Button("Set all draw ON"))
                {
                    for (auto& [ID, bucket] : g_buckets)
                    {
                        bucket.draw = true;
                    }
                }

                if (ImGui::Button("Set all draw OFF"))
                {
                    for (auto& [ID, bucket] : g_buckets)
                    {
                        bucket.draw = false;
                    }
                }

                if (ImGui::TreeNode("Cave buckets"))
                {
                    ImGui::Text("buckets = %zu", g_buckets.size());

                    for (auto& [ID, bucket] : g_buckets)
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
                                    ImGui::Text("VAO = %u, VBO = %u, points = %zu", current->vao, current->vbo, current->points.size());

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

        if (g_enable_icp_optimization_for_autoplay && (g_trajectory_index_auto_play || ImGui::Button("Single ICP iteration")))
        {
            auto start = std::chrono::high_resolution_clock::now();

            const auto& in_obb_ids           = in_obb_ids_in_obb_proximity.first;
            const auto& in_obb_proximity_ids = in_obb_ids_in_obb_proximity.second;

            std::vector<Eigen::Vector3d> target         = {};
            std::vector<Eigen::Vector3d> target_normals = {};

            for (const auto& in_obb_id : in_obb_ids)
            {
                PointCloudLOD* last_lod = g_buckets[in_obb_id].lods;

                while (last_lod->next)
                {
                    last_lod = last_lod->next;
                }

                for (const auto& point : last_lod->points)
                {
                    const glm::vec3 normal_reoriented = reorient(point.normal, point.position, stretcher_position);
                    target.emplace_back(static_cast<double>(point.position.x), static_cast<double>(point.position.y), static_cast<double>(point.position.z));
                    target_normals.emplace_back(static_cast<double>(normal_reoriented.x), static_cast<double>(normal_reoriented.y), static_cast<double>(normal_reoriented.z));
                }
            }

            for (const auto& in_obb_proximity_id : in_obb_proximity_ids)
            {
                PointCloudLOD* last_lod = g_buckets[in_obb_proximity_id].lods;

                while (last_lod->next)
                {
                    last_lod = last_lod->next;
                }

                for (const auto& point : last_lod->points)
                {
                    const glm::vec3 normal_reoriented = reorient(point.normal, point.position, stretcher_position);
                    target.emplace_back(static_cast<double>(point.position.x), static_cast<double>(point.position.y), static_cast<double>(point.position.z));
                    target_normals.emplace_back(static_cast<double>(normal_reoriented.x), static_cast<double>(normal_reoriented.y), static_cast<double>(normal_reoriented.z));
                }
            }

            const glm::dmat4 stretcher_pose_double = glm::dmat4(stretcher_pose);
            Eigen::Affine3d  m_pose_result         = glm_mat4_to_eigen_affine_3d(stretcher_pose_double);

            static Eigen::Affine3d m_previous_pose = m_pose_result;

#define DEBUG_LOG 0

#if DEBUG_LOG
            spdlog::warn("ICP :");
            spdlog::warn(" - input glm pose   : {}", glm_dmat4_to_string(stretcher_pose_double));
            spdlog::warn(" - input eigen pose : {}", eigen_affine3d_to_string(m_pose_result));
            spdlog::warn(" - source points : {}", g_stretcher_icp_source.size());
            spdlog::warn(" - target points : {}", target.size());
#endif

            auto ppose = m_pose_result;

            PairWiseICP icp{};
            icp.compute(g_stretcher_icp_source, target, target_normals, g_icp_search_radious, g_icp_number_of_iterations, m_pose_result, g_icp_is_repulsion, m_previous_pose);

            //  static int32_t indx = 0;
            //
            //  if (std::ofstream ofs = std::ofstream(std::to_string(indx++) + ".txt", std::ios::out))
            //  {
            //      if (target.size() != target_normals.size())
            //      {
            //          std::exit(2);
            //      }
            //
            //      for (size_t i = 0; i < target.size(); i++)
            //      {
            //          ofs << target[i].x() << ' ' << target[i].y() << ' ' << target[i].z() << ' ' << target_normals[i].x() << ' ' << target_normals[i].y() << ' ' << target_normals[i].z() << '\n';
            //      }
            //  }

            double dist = (m_pose_result.translation() - m_previous_pose.translation()).norm();

            if (dist > 1.0)
            {
                m_previous_pose = ppose;
            }

#if DEBUG_LOG
            spdlog::warn("ICP result:");
            spdlog::warn(" - input glm pose   : {}", glm_dmat4_to_string(stretcher_pose_double));
            spdlog::warn(" - output eigen pose : {}", eigen_affine3d_to_string(m_pose_result));
#endif

            if (g_trajectory_index_auto_play)
            {
                const Eigen::Affine3f m_pose_result_float = m_pose_result.cast<float>();

                const glm::mat4 m_pose_result_float_glm       = glm::make_mat4(m_pose_result_float.data());
                const auto      m_pose_result_float_glm_trans = glm::vec3(m_pose_result_float_glm[3]);
                const auto      m_pose_result_float_glm_rot   = glm::mat3(m_pose_result_float_glm);

                g_optimized_trajectory_positions.push_back(Point{m_pose_result_float_glm_trans});
                g_optimized_trajectory_orientations_mat33.push_back(TrajectoryPoseOrientationMat33{m_pose_result_float_glm_rot});

                // g_optimized_trajectory_poses.push_back(m_pose_result);

                const Eigen::Vector3f position = m_pose_result_float.translation().eval();

                glNamedBufferSubData(g_optimized_trajectory_positions_vbo, sizeof(Eigen::Vector3f) * (g_optimized_trajectory_positions.size() - 1), sizeof(Eigen::Vector3f), position.data());

                /// TEST
                {
                    const auto new_stretcher_pose = m_pose_result_float.matrix().eval();

                    std::memcpy(glm::value_ptr(stretcher_pose), new_stretcher_pose.data(), sizeof(float) * 16);
                }
            }

            auto end = std::chrono::high_resolution_clock::now();

            g_cpu_time_icp_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1'000'000.0f;
        }

        ImGui::Checkbox("g_modify_current_pose_with_gizmo", &g_modify_current_pose_with_gizmo);

        ImGui::End();

        if (g_modify_current_pose_with_gizmo)
        {
            glm::vec3 stretcher_position    = glm::vec3(0.0f);
            glm::mat3 stretcher_orientation = glm::mat3(1.0f);

            if (g_trajectory_positions.size() && g_trajectory_orientations_mat33.size())
            {
                const auto& trajectory_point      = g_trajectory_positions[g_trajectory_index];
                const auto& trajectoryorientation = g_trajectory_orientations_mat33[g_trajectory_index];

                stretcher_position    = trajectory_point.position;
                stretcher_orientation = trajectoryorientation.orientation;
            }

            glm::mat4 stretcher_pose = glm::translate(glm::mat4(1.0f), stretcher_position) * glm::mat4(stretcher_orientation);

            ImGuizmo::BeginFrame();
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist(ImGui::GetBackgroundDrawList());
            ImGuizmo::SetRect(0, 0, width, height);

            float objectMatrix[16] =
                {1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1};

            std::memcpy(objectMatrix, glm::value_ptr(stretcher_pose), sizeof(glm::mat4));

            ImGuizmo::Manipulate(
                glm::value_ptr(view),
                glm::value_ptr(projection),
                ImGuizmo::TRANSLATE | ImGuizmo::ROTATE,
                ImGuizmo::LOCAL,
                objectMatrix);

            const auto modified = glm::mat4(
                objectMatrix[0], objectMatrix[1], objectMatrix[2], objectMatrix[3],
                objectMatrix[4], objectMatrix[5], objectMatrix[6], objectMatrix[7],
                objectMatrix[8], objectMatrix[9], objectMatrix[10], objectMatrix[11],
                objectMatrix[12], objectMatrix[13], objectMatrix[14], objectMatrix[15]);

            glm::vec3 position = glm::vec3(modified[3]);
            glm::mat3 rotation = glm::mat3(modified);

            if (g_trajectory_positions.size() && g_trajectory_orientations_mat33.size())
            {
                g_trajectory_positions[g_trajectory_index].position             = position;
                g_trajectory_orientations_mat33[g_trajectory_index].orientation = rotation;

                glNamedBufferSubData(g_trajectory_positions_vbo, sizeof(Eigen::Vector3f) * g_trajectory_index, sizeof(Eigen::Vector3f), &g_trajectory_positions[g_trajectory_index].position);
            }
        }

        ImGui::Render();

        // PICKING POINT CLOUD
        {
            static bool prev_mouse_pressed = false;

            bool ctrl_pressed  = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);
            bool mouse_pressed = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);

            // Trigger once when mouse goes from released -> pressed, while Ctrl is held
            bool new_click     = mouse_pressed && !prev_mouse_pressed && ctrl_pressed;
            prev_mouse_pressed = mouse_pressed;

            if (new_click)
            {
                PointCloudRecord* picked_record = nullptr;
                glm::ivec3        picked_id{};
                glm::vec3         camera_pos     = camera.position;
                glm::vec3         camera_forward = glm::normalize(camera.target - camera.position);

                double mouse_x, mouse_y;
                glfwGetCursorPos(window, &mouse_x, &mouse_y);

                int width, height;
                glfwGetFramebufferSize(window, &width, &height);

                float x_ndc = (2.0f * static_cast<float>(mouse_x) / width) - 1.0f;
                float y_ndc = 1.0f - (2.0f * static_cast<float>(mouse_y) / height);

                glm::vec4 ray_clip(x_ndc, y_ndc, -1.0f, 1.0f);
                glm::vec4 ray_eye = glm::inverse(projection) * ray_clip;
                ray_eye.z         = -1.0f;
                ray_eye.w         = 0.0f;

                glm::vec3 ray_dir      = glm::normalize(glm::vec3(glm::inverse(view) * ray_eye));
                float     closest_dist = std::numeric_limits<float>::max();

                const float PICK_RADIUS = 0.1f;

                for (auto& [ID, bucket] : g_buckets)
                {
                    glm::vec3 center    = 0.5f * (bucket.aabb.min + bucket.aabb.max);
                    glm::vec3 to_center = center - camera_pos;

                    if (glm::dot(to_center, camera_forward) <= 0.0f)
                    {
                        continue;
                    }

                    if (g_use_fine_picking)
                    {
                        PointCloudLOD* lod = bucket.lods;
                        while (lod->next)
                        {
                            lod = lod->next;
                        }

                        // check all points in last LOD
                        for (const auto& p : lod->points)
                        {
                            const glm::vec3& point    = p.position;
                            glm::vec3        diff     = point - camera_pos;
                            float            proj_len = glm::dot(diff, ray_dir);

                            if (proj_len >= closest_dist)
                                continue;

                            glm::vec3 closest_point = camera_pos + ray_dir * proj_len;
                            float     dist_to_ray   = glm::length(point - closest_point);

                            if (dist_to_ray <= PICK_RADIUS)
                            {
                                closest_dist  = proj_len;
                                picked_record = &bucket;
                                picked_id     = ID;
                                break;
                            }
                        }
                    }
                    else
                    {
                        // simple bounding-box picking using record extent
                        glm::vec3 bmin = bucket.aabb.min;
                        glm::vec3 bmax = bucket.aabb.max;

                        float tmin = 0.0f, tmax = 0.0f;

                        for (int i = 0; i < 3; ++i)
                        {
                            if (std::abs(ray_dir[i]) < 1e-6f)
                            {
                                if (camera_pos[i] < bmin[i] || camera_pos[i] > bmax[i])
                                {
                                    tmin = tmax = -1.0f;
                                    break;
                                }
                            }
                            else
                            {
                                float invD = 1.0f / ray_dir[i];
                                float t0   = (bmin[i] - camera_pos[i]) * invD;
                                float t1   = (bmax[i] - camera_pos[i]) * invD;
                                if (t0 > t1)
                                    std::swap(t0, t1);
                                tmin = (i == 0) ? t0 : std::max(tmin, t0);
                                tmax = (i == 0) ? t1 : std::min(tmax, t1);
                            }
                        }

                        if (tmax >= tmin && tmin >= 0.0f && tmin < closest_dist)
                        {
                            closest_dist  = tmin;
                            picked_record = &bucket;
                            picked_id     = ID;
                        }
                    }
                }

                if (picked_record)
                {
                    spdlog::info("Picking hit : ID = [{} {} {}]", picked_id.x, picked_id.y, picked_id.z);

                    glm::vec3 center = picked_record->aabb.min + 0.5f * (picked_record->aabb.max - picked_record->aabb.min);
                    glm::vec3 offset = camera.position - camera.target;
                    camera.target    = center;
                    camera.position  = camera.target + offset;
                }
                else
                {
                    spdlog::warn("Picking missed ...");
                }
            }
        }

        // ORIGIN
        if (g_draw_origin)
        {
            glLineWidth(g_origin_width);
            glUseProgram(origin_program);
            glProgramUniformMatrix4fv(origin_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));
            glProgramUniform1f(origin_program, 1, g_origin_scale);
            glBindVertexArray(origin_vao);
            glDrawArrays(GL_LINES, 0, 6);
            glLineWidth(1.0f);
        }

        // CAMERA_TARGET
        if (g_draw_camera_target)
        {
            glLineWidth(g_target_width);
            glUseProgram(camera_target_program);
            glProgramUniformMatrix4fv(camera_target_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));
            glProgramUniform3fv(camera_target_program, 1, 1, glm::value_ptr(camera.target));
            glProgramUniform1f(camera_target_program, 2, g_target_scale);
            glProgramUniform3fv(camera_target_program, 3, 1, glm::value_ptr(g_target_color));
            glBindVertexArray(target_vao);
            glDrawArrays(GL_LINES, 0, 6);
            glLineWidth(1.0f);
        }

        // TRAJECTORY
        if (g_draw_trajectory && can_draw_trajectory)
        {
            auto start = std::chrono::high_resolution_clock::now();

            glLineWidth(g_trajectory_width);
            glUseProgram(trajectory_program);
            glProgramUniformMatrix4fv(trajectory_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));
            glProgramUniform3fv(trajectory_program, 1, 1, glm::value_ptr(g_trajectory_color));
            glBindVertexArray(g_trajectory_positions_vao);
            glDrawArrays(GL_LINE_STRIP, 0, g_trajectory_positions.size());
            glLineWidth(1.0f);

            auto end = std::chrono::high_resolution_clock::now();

            g_cpu_time_draw_trajectory_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1'000'000.0f;
        }

        // OPTIMIZED TRAJECTORY
        if (g_draw_optimized_trajectory && can_draw_optimized_trajectory)
        {
            glLineWidth(g_optimized_trajectory_width);
            glUseProgram(trajectory_program);
            glProgramUniformMatrix4fv(trajectory_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));
            glProgramUniform3fv(trajectory_program, 1, 1, glm::value_ptr(g_optimized_trajectory_color));
            glBindVertexArray(g_optimized_trajectory_positions_vao);
            glDrawArrays(GL_LINE_STRIP, 0, g_optimized_trajectory_positions.size());
            glLineWidth(1.0f);
        }

        //  STRETCHER
        if (g_draw_stretcher && can_draw_stretcher)
        {
            auto start = std::chrono::high_resolution_clock::now();

            glUseProgram(stretcher_program);
            glProgramUniformMatrix4fv(stretcher_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));
            glProgramUniformMatrix4fv(stretcher_program, 1, 1, GL_FALSE, glm::value_ptr(stretcher_pose));
            glBindVertexArray(g_stretcher_vao);
            glDrawElements(GL_TRIANGLES, g_stretcher_indices.size(), GL_UNSIGNED_INT, nullptr);

            auto end = std::chrono::high_resolution_clock::now();

            g_cpu_time_draw_stretcher_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1'000'000.0f;
        }

        //  STRETCHER BBOX
        if (g_draw_stretcher_bbox && can_draw_stretcher)
        {
            glLineWidth(g_stretcher_box_width);
            glUseProgram(bounding_box_stretcher_program);
            glProgramUniformMatrix4fv(bounding_box_stretcher_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));
            glProgramUniform3fv(bounding_box_stretcher_program, 1, 1, glm::value_ptr(g_stretcher_box_color));
            glProgramUniformMatrix4fv(bounding_box_stretcher_program, 2, 1, GL_FALSE, glm::value_ptr(stretcher_pose));
            glBindVertexArray(g_stretcher_aabb_vao);
            glDrawArrays(GL_LINES, 0, 24);
            glLineWidth(1.0f);
        }

        const bool draw_any_cave_lod = g_point_cloud_bucket_draw || g_point_cloud_bucket_in_obb_draw || g_point_cloud_bucket_in_obb_proximity_draw;

        // POINT_CLOUD
        if (g_draw_point_cloud && can_draw_cave && draw_any_cave_lod)
        {
            auto start = std::chrono::high_resolution_clock::now();

            glm::vec3 camera_pos = glm::vec3(glm::inverse(view)[3]);

            glPointSize(g_point_cloud_point_size);
            glUseProgram(point_cloud_program);
            glProgramUniformMatrix4fv(point_cloud_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));

            for (auto& [ID, bucket] : g_buckets)
            {
                if (!bucket.draw || !bucket.lods)
                {
                    continue;
                }

                glm::vec3 center   = 0.5f * (bucket.aabb.min + bucket.aabb.max);
                float     distance = glm::length(center - camera_pos);

                size_t lod_count = 0;
                for (PointCloudLOD* lod = bucket.lods; lod; lod = lod->next)
                {
                    ++lod_count;
                }

                size_t         lod_index = lod_from_distance(distance, 70.0f, lod_count);
                PointCloudLOD* lod       = get_lod_at_index(&bucket, lod_index);

                if (!lod || !lod_in_camera_frustum(*lod, frustum))
                {
                    continue;
                }

                const bool is_in_obb           = std::ranges::contains(in_obb_ids_in_obb_proximity.first, ID);
                const bool is_on_obb_proximity = std::ranges::contains(in_obb_ids_in_obb_proximity.second, ID);

                if (is_in_obb && g_point_cloud_bucket_in_obb_draw)
                {
                    glBindVertexArray(lod->vao);
                    glDrawArrays(GL_POINTS, 0, lod->points.size());
                    continue;
                }

                if (is_on_obb_proximity && g_point_cloud_bucket_in_obb_proximity_draw)
                {
                    glBindVertexArray(lod->vao);
                    glDrawArrays(GL_POINTS, 0, lod->points.size());
                    continue;
                }

                if (g_point_cloud_bucket_draw && !(is_in_obb || is_on_obb_proximity))
                {
                    glBindVertexArray(lod->vao);
                    glDrawArrays(GL_POINTS, 0, lod->points.size());
                    continue;
                }
            }

            glPointSize(1.0f);

            auto end = std::chrono::high_resolution_clock::now();

            g_cpu_time_draw_cave_buckets_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1'000'000.0f;
        }

        const bool draw_any_cave_boxes = g_point_cloud_bbox_draw || g_point_cloud_bbox_in_obb_draw || g_point_cloud_bbox_in_obb_proximity_draw;

        // POINT CLOUD BOXES
        if (g_draw_bounding_box && can_draw_bounding_boxes && (draw_any_cave_boxes))
        {
            auto start = std::chrono::high_resolution_clock::now();

            glm::vec3 camera_pos = glm::vec3(glm::inverse(view)[3]);

            glUseProgram(bounding_box_program);
            glProgramUniformMatrix4fv(bounding_box_program, 0, 1, GL_FALSE, glm::value_ptr(MVP));

            for (auto& [ID, bucket] : g_buckets)
            {
                if (!record_in_camera_frustum(bucket, frustum))
                {
                    continue;
                }

                const bool is_in_obb           = std::ranges::contains(in_obb_ids_in_obb_proximity.first, ID);
                const bool is_on_obb_proximity = std::ranges::contains(in_obb_ids_in_obb_proximity.second, ID);

                if (is_in_obb && g_point_cloud_bbox_in_obb_draw)
                {
                    glLineWidth(g_point_cloud_bbox_in_obb_width);
                    glm::vec3 red(1.0f, 0.0f, 0.0f);
                    glProgramUniform3fv(bounding_box_program, 1, 1, glm::value_ptr(red));

                    glBindVertexArray(bucket.bbox_vao);
                    glDrawArrays(GL_LINES, 0, 24);

                    glLineWidth(1.0f);

                    continue;
                }
                else if (is_on_obb_proximity && g_point_cloud_bbox_in_obb_proximity_draw)
                {
                    glLineWidth(g_point_cloud_bbox_in_obb_proximity_width);
                    glm::vec3 blue(0.0f, 0.0f, 1.0f);
                    glProgramUniform3fv(bounding_box_program, 1, 1, glm::value_ptr(blue));

                    glBindVertexArray(bucket.bbox_vao);
                    glDrawArrays(GL_LINES, 0, 24);

                    glLineWidth(1.0f);

                    continue;
                }
                else if (g_point_cloud_bbox_draw && !(is_in_obb || is_on_obb_proximity))
                {
                    glLineWidth(g_point_cloud_bbox_width);
                    glm::vec3 white(1.0f, 1.0f, 1.0f);
                    glProgramUniform3fv(bounding_box_program, 1, 1, glm::value_ptr(white));

                    glBindVertexArray(bucket.bbox_vao);
                    glDrawArrays(GL_LINES, 0, 24);

                    glLineWidth(1.0f);

                    continue;
                }
            }

            auto end = std::chrono::high_resolution_clock::now();

            g_cpu_time_draw_cave_buckets_bbox_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1'000'000.0f;
        }

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}