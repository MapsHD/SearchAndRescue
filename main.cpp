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
#include <cave-traversal-tool/Structures.h>

#include <cave-traversal-tool/OpenGL/Buffer.h>
#include <cave-traversal-tool/OpenGL/Program.h>
#include <cave-traversal-tool/OpenGL/VertexArray.h>

#include <cave-traversal-tool/Processing.h>

#include <cave-traversal-tool/Shaders.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <ImGuizmo.h>

#include <portable-file-dialogs.h>

#define CHECK_BOOL(expression)                                    \
    do                                                            \
    {                                                             \
        if (!expression)                                          \
        {                                                         \
            spdlog::error("Expression failed : {}", #expression); \
            std::abort();                                         \
        }                                                         \
    } while (0)

static std::vector<PointIntensity> g_cave_vertices{};
static PointCloudBucket            g_buckets{};

static Buffer*      g_stretcher_aabb_vbo = nullptr;
static VertexArray* g_stretcher_aabb_vao = nullptr;
static AABB         g_stretcher_aabb{};

static std::vector<ColorPoint> g_stretcher_vertices{};
static std::vector<uint32_t>   g_stretcher_indices{};

static std::vector<Point>                          g_trajectory_positions{};
static std::vector<TrajectoryPoseOrientationMat33> g_trajectory_orientations_mat33{};

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

static bool g_draw_origin         = true;
static bool g_draw_camera_target  = true;
static bool g_draw_trajectory     = true;
static bool g_draw_stretcher      = true;
static bool g_draw_stretcher_bbox = true;
static bool g_draw_point_cloud    = true;
static bool g_draw_bounding_box   = true;

static glm::vec3 g_clear_color = {0.2f, 0.2f, 0.2f};

static float g_origin_scale = 1.0f;
static float g_origin_width = 1.0f;

static float     g_target_scale = 1.0f;
static float     g_target_width = 1.0f;
static glm::vec3 g_target_color = {1.0f, 1.0f, 1.0f};

static float     g_trajectory_width = 1.0f;
static glm::vec3 g_trajectory_color = {1.0f, 1.0f, 1.0f};

static glm::vec3 g_stretcher_box_color = {0.0f, 1.0f, 1.0f};
static float     g_stretcher_box_width = 1.0f;

static float g_point_cloud_point_size = 1.5f;

static float g_point_cloud_bbox_width                  = 1.0f;
static float g_point_cloud_bbox_in_obb_width           = 4.0f;
static float g_point_cloud_bbox_in_obb_proximity_width = 3.0f;

static bool g_point_cloud_bbox_draw                  = false;
static bool g_point_cloud_bbox_in_obb_draw           = true;
static bool g_point_cloud_bbox_in_obb_proximity_draw = true;

static bool g_point_cloud_bucket_draw                  = false;
static bool g_point_cloud_bucket_in_obb_draw           = true;
static bool g_point_cloud_bucket_in_obb_proximity_draw = true;

static bool    g_use_fixed_lod   = false;
static int32_t g_fixed_lod_index = 0;

static Buffer*      g_stretcher_vbo          = nullptr;
static Buffer*      g_stretcher_index_buffer = nullptr;
static VertexArray* g_stretcher_vao          = nullptr;

static Buffer*      g_trajectory_positions_vbo = nullptr;
static VertexArray* g_trajectory_positions_vao = nullptr;

//
static bool     g_trajectory_index_auto_play           = false;
static int32_t  g_trajectory_index_auto_play_increment = 1;
static uint32_t g_trajectory_index                     = 0;

// gizmo
static bool g_modify_current_pose_with_gizmo = false;

static inline bool open_single_file_with_pfd(const std::string& title, const std::string& filter_description, const std::string& filter, std::string& out)
{
    const std::vector<std::string> result = pfd::open_file(
                                                title,
                                                "",
                                                {filter_description, filter})
                                                .result();

    if (result.empty())
    {
        spdlog::error("PFD result emplty - doing nothing ...");
        return false;
    }

    if (result.size() > 1)
    {
        spdlog::warn("PFD result contains multiple entiries - loading first ...");
    }

    out = result.front();

    return true;
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

    if (g_trajectory_positions_vao)
    {
        spdlog::debug("Deleting old trajectory positions VAO : {}", g_trajectory_positions_vao->GetID());
        delete g_trajectory_positions_vao;
    }

    if (g_trajectory_positions_vbo)
    {
        spdlog::debug("Deleting old trajectory positions VBO : {}", g_trajectory_positions_vbo->GetID());
        delete g_trajectory_positions_vbo;
    }

    g_trajectory_positions_vbo = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(g_trajectory_positions), g_trajectory_positions.data());
    g_trajectory_positions_vao = new VertexArray(g_trajectory_positions_vbo, false, nullptr, false, layout_point);

    spdlog::debug("Created VAO [{}] and VBO [{}]", g_trajectory_positions_vao->GetID(), g_trajectory_positions_vbo->GetID());
}

static void rebuild_stretcher_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_color_point = opengl_vertex_array_get_vertex_layout<ColorPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point       = opengl_vertex_array_get_vertex_layout<Point>();

    if (g_stretcher_aabb_vao)
    {
        spdlog::debug("Deleting old stretcher AABB positions VAO : {}", g_stretcher_aabb_vao->GetID());
        delete g_stretcher_aabb_vao;
    }

    if (g_stretcher_aabb_vbo)
    {
        spdlog::debug("Deleting old stretcher AABB positions VBO : {}", g_stretcher_aabb_vbo->GetID());
        delete g_stretcher_aabb_vbo;
    }

    if (g_stretcher_vao)
    {
        spdlog::debug("Deleting old stretcher positions VAO : {}", g_stretcher_vao->GetID());
        delete g_stretcher_vao;
    }

    if (g_stretcher_vbo)
    {
        spdlog::debug("Deleting old stretcher positions VBO : {}", g_stretcher_vbo->GetID());
        delete g_stretcher_vbo;
    }

    if (g_stretcher_index_buffer)
    {
        spdlog::debug("Deleting old stretcher index IBO : {}", g_stretcher_index_buffer->GetID());
        delete g_stretcher_index_buffer;
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

    g_stretcher_aabb_vbo = new Buffer(GL_NONE, std_vector_size(line_vertices), line_vertices.data());
    g_stretcher_aabb_vao = new VertexArray(g_stretcher_aabb_vbo, false, nullptr, false, layout_point);

    g_stretcher_vbo          = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(g_stretcher_vertices), g_stretcher_vertices.data());
    g_stretcher_index_buffer = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(g_stretcher_indices), g_stretcher_indices.data());

    g_stretcher_vao = new VertexArray(g_stretcher_vbo, false, g_stretcher_index_buffer, false, layout_color_point);

    spdlog::debug("Created VAO [{}], VBO [{}] and IBO [{}]", g_stretcher_vao->GetID(), g_stretcher_vbo->GetID(), g_stretcher_index_buffer->GetID());
}

static void rebuild_cave_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_point_intensity = opengl_vertex_array_get_vertex_layout<PointIntensity>();
    const std::vector<VertexBufferAttributeLayout> layout_point           = opengl_vertex_array_get_vertex_layout<Point>();

    // Clear old data
    for (auto& [ID, bucket] : g_buckets)
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
}

static inline void load_trajectory()
{
    std::string filename;

    if (open_single_file_with_pfd("Open CSV file", "CSV Files (.csv)", "*.csv", filename))
    {
        CHECK_BOOL(load_trajectory_csv(filename, g_trajectory_positions, g_trajectory_orientations_mat33, g_load_csv_every_nth));
        rebuild_trajectory_mat33_opengl_data();
    }
}

static inline void load_object()
{
    std::string filename;

    if (open_single_file_with_pfd("Open PLY file", "PLY Files (.ply)", "*.ply", filename))
    {
        CHECK_BOOL(load_stretcher_ply(filename, g_stretcher_vertices, g_stretcher_indices));
        rebuild_stretcher_opengl_data();
    }
}

static inline void load_environment()
{
    std::string filename;

    if (open_single_file_with_pfd("Open LAZ file", "LAZ Files (*.laz *.las)", "*.laz *.las", filename))
    {
        CHECK_BOOL(load_cave_laz(filename, g_cave_vertices));
        rebuild_cave_opengl_data();
    }
}

static inline Program* make_program(const ProgramShaderSources& sources)
{
    return new Program(
        {ShaderDescriptor{
             .shader_type = GL_VERTEX_SHADER,
             .source_size = sources.vertex_source_size,
             .source      = sources.vertex_source},
         ShaderDescriptor{
             .shader_type = GL_FRAGMENT_SHADER,
             .source_size = sources.fragment_source_size,
             .source      = sources.fragment_source}});
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

    glfwSwapInterval(1);

    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    // glEnable(GL_DEBUG_OUTPUT);
    // glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    // glDebugMessageCallback(ErrorCallback::OpenGL, nullptr);

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    Program* origin_program                 = make_program(GetProgramShaderSources_Origin());
    Program* camera_target_program          = make_program(GetProgramShaderSources_CameraTarger());
    Program* point_cloud_program            = make_program(GetProgramShaderSources_PointCloud());
    Program* trajectory_program             = make_program(GetProgramShaderSources_Trajectory());
    Program* stretcher_program              = make_program(GetProgramShaderSources_Stretcher());
    Program* bounding_box_program           = make_program(GetProgramShaderSources_BoundingBox());
    Program* bounding_box_stretcher_program = make_program(GetProgramShaderSources_BoundingBoxStretcher());

    const std::vector<VertexBufferAttributeLayout> layout_color_point = opengl_vertex_array_get_vertex_layout<ColorPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point       = opengl_vertex_array_get_vertex_layout<Point>();

    Buffer*      origin_buffer = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(origin), origin.data());
    VertexArray* origin_vao    = new VertexArray(origin_buffer, false, nullptr, false, layout_color_point);

    Buffer*      target_buffer = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(target), target.data());
    VertexArray* target_vao    = new VertexArray(target_buffer, false, nullptr, false, layout_point);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

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

        const bool can_draw_trajectory     = g_trajectory_positions.size() && g_trajectory_orientations_mat33.size();
        const bool can_draw_stretcher      = g_stretcher_vertices.size() && g_stretcher_indices.size();
        const bool can_draw_cave           = g_buckets.size();
        const bool can_draw_bounding_boxes = g_buckets.size();

        std::array<glm::vec4, 6> frustum{};
        compute_camera_frustum_planes(view, projection, frustum);

        size_t max_lod_count = 0;
        for (auto& [ID, bucket] : g_buckets)
        {
            size_t bucket_lod_count = 0;
            for (PointCloudLOD* lod = bucket.lods; lod; lod = lod->next)
            {
                ++bucket_lod_count;
            }

            max_lod_count = std::max(max_lod_count, bucket_lod_count);
        }

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
                ImGui::Text("g_cpu_time_draw_trajectory_ms        : %.3f ms", g_cpu_time_draw_trajectory_ms);
                ImGui::Text("g_cpu_time_draw_stretcher_ms         : %.3f ms", g_cpu_time_draw_stretcher_ms);
                ImGui::Text("g_cpu_time_draw_cave_buckets_ms      : %.3f ms", g_cpu_time_draw_cave_buckets_ms);
                ImGui::Text("g_cpu_time_draw_cave_buckets_bbox_ms : %.3f ms", g_cpu_time_draw_cave_buckets_bbox_ms);
                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::TreeNode("File input / output"))
            {
                if (ImGui::BeginTable("IOGrid", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchSame))
                {
                    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                    ImGui::TableSetupColumn("Read", ImGuiTableColumnFlags_WidthStretch);
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

                    // Object
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Object");

                    ImGui::TableSetColumnIndex(1);
                    if (ImGui::Button("##obj_read", ImVec2(-FLT_MIN, 0)))
                    {
                        load_object();
                    }

                    // Environment
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Environment");

                    ImGui::TableSetColumnIndex(1);
                    if (ImGui::Button("##env_read", ImVec2(-FLT_MIN, 0)))
                    {
                        load_environment();
                    }

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
            if (ImGui::TreeNode("Level of Detail (LOD)"))
            {
                ImGui::Checkbox("g_use_fixed_lod", &g_use_fixed_lod);

                const int32_t max_lod_index = max_lod_count > 0 ? static_cast<int32_t>(max_lod_count - 1) : 0;
                ImGui::BeginDisabled(!g_use_fixed_lod);
                ImGui::SliderInt("g_fixed_lod_index", &g_fixed_lod_index, 0, max_lod_index);
                ImGui::EndDisabled();
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

                ImGui::ColorEdit3("g_clear_color", glm::value_ptr(g_clear_color));
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

                glNamedBufferSubData(g_trajectory_positions_vbo->GetID(), sizeof(glm::vec3) * g_trajectory_index, sizeof(glm::vec3), &g_trajectory_positions[g_trajectory_index].position);
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
            origin_program->Bind();
            origin_program->PushUniform16F32("u_MVP", MVP);
            origin_program->PushUniform1F32("u_Scale", g_origin_scale);
            origin_vao->Bind();
            origin_vao->DrawArray(GL_LINES, 6);
            glLineWidth(1.0f);
        }

        // CAMERA_TARGET
        if (g_draw_camera_target)
        {
            glLineWidth(g_target_width);
            camera_target_program->Bind();
            camera_target_program->PushUniform16F32("u_MVP", MVP);
            camera_target_program->PushUniform3F32("u_Translation", camera.target);
            camera_target_program->PushUniform1F32("u_Scale", g_target_scale);
            camera_target_program->PushUniform3F32("u_Color", g_target_color);
            target_vao->Bind();
            target_vao->DrawArray(GL_LINES, 6);
            glLineWidth(1.0f);
        }

        // TRAJECTORY
        if (g_draw_trajectory && can_draw_trajectory)
        {
            auto start = std::chrono::high_resolution_clock::now();

            glLineWidth(g_trajectory_width);
            trajectory_program->Bind();
            trajectory_program->PushUniform16F32("u_MVP", MVP);
            trajectory_program->PushUniform3F32("u_Color", g_trajectory_color);
            g_trajectory_positions_vao->Bind();
            g_trajectory_positions_vao->DrawArray(GL_LINE_STRIP, g_trajectory_positions.size());
            glLineWidth(1.0f);

            auto end = std::chrono::high_resolution_clock::now();

            g_cpu_time_draw_trajectory_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1'000'000.0f;
        }

        //  STRETCHER
        if (g_draw_stretcher && can_draw_stretcher)
        {
            auto start = std::chrono::high_resolution_clock::now();

            stretcher_program->Bind();
            stretcher_program->PushUniform16F32("u_MVP", MVP);
            stretcher_program->PushUniform16F32("u_Pose", stretcher_pose);

            g_stretcher_vao->Bind();
            g_stretcher_vao->DrawElements(GL_TRIANGLES, g_stretcher_indices.size(), 1, 0);

            auto end = std::chrono::high_resolution_clock::now();

            g_cpu_time_draw_stretcher_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1'000'000.0f;
        }

        //  STRETCHER BBOX
        if (g_draw_stretcher_bbox && can_draw_stretcher)
        {
            glLineWidth(g_stretcher_box_width);

            bounding_box_stretcher_program->Bind();
            bounding_box_stretcher_program->PushUniform16F32("u_MVP", MVP);
            bounding_box_stretcher_program->PushUniform3F32("u_Color", g_stretcher_box_color);
            bounding_box_stretcher_program->PushUniform16F32("u_Pose", stretcher_pose);
            g_stretcher_aabb_vao->Bind();
            g_stretcher_aabb_vao->DrawArray(GL_LINES, 24);
            glLineWidth(1.0f);
        }

        const bool draw_any_cave_lod = g_point_cloud_bucket_draw || g_point_cloud_bucket_in_obb_draw || g_point_cloud_bucket_in_obb_proximity_draw;

        // POINT_CLOUD
        if (g_draw_point_cloud && can_draw_cave && draw_any_cave_lod)
        {
            auto start = std::chrono::high_resolution_clock::now();

            glm::vec3 camera_pos = glm::vec3(glm::inverse(view)[3]);

            glPointSize(g_point_cloud_point_size);

            point_cloud_program->Bind();
            point_cloud_program->PushUniform16F32("u_MVP", MVP);

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

                size_t         lod_index = g_use_fixed_lod ? static_cast<size_t>(g_fixed_lod_index) : lod_from_distance(distance, 70.0f, lod_count);
                PointCloudLOD* lod       = get_lod_at_index(&bucket, lod_index);

                if (!lod || !lod_in_camera_frustum(*lod, frustum))
                {
                    continue;
                }

                const bool is_in_obb           = std::ranges::contains(in_obb_ids_in_obb_proximity.first, ID);
                const bool is_on_obb_proximity = std::ranges::contains(in_obb_ids_in_obb_proximity.second, ID);

                if (is_in_obb && g_point_cloud_bucket_in_obb_draw)
                {
                    lod->vao->Bind();
                    lod->vao->DrawArray(GL_POINTS, lod->points.size());
                    continue;
                }

                if (is_on_obb_proximity && g_point_cloud_bucket_in_obb_proximity_draw)
                {
                    lod->vao->Bind();
                    lod->vao->DrawArray(GL_POINTS, lod->points.size());
                    continue;
                }

                if (g_point_cloud_bucket_draw && !(is_in_obb || is_on_obb_proximity))
                {
                    lod->vao->Bind();
                    lod->vao->DrawArray(GL_POINTS, lod->points.size());
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

            bounding_box_program->Bind();
            bounding_box_program->PushUniform16F32("u_MVP", MVP);

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

                    bounding_box_program->PushUniform3F32("u_Color", red);

                    bucket.bbox_vao->Bind();
                    bucket.bbox_vao->DrawArray(GL_LINES, 24);

                    glLineWidth(1.0f);

                    continue;
                }
                else if (is_on_obb_proximity && g_point_cloud_bbox_in_obb_proximity_draw)
                {
                    glLineWidth(g_point_cloud_bbox_in_obb_proximity_width);
                    glm::vec3 blue(0.0f, 0.0f, 1.0f);

                    bounding_box_program->PushUniform3F32("u_Color", blue);

                    bucket.bbox_vao->Bind();
                    bucket.bbox_vao->DrawArray(GL_LINES, 24);

                    glLineWidth(1.0f);

                    continue;
                }
                else if (g_point_cloud_bbox_draw && !(is_in_obb || is_on_obb_proximity))
                {
                    glLineWidth(g_point_cloud_bbox_width);
                    glm::vec3 white(1.0f, 1.0f, 1.0f);

                    bounding_box_program->PushUniform3F32("u_Color", white);

                    bucket.bbox_vao->Bind();
                    bucket.bbox_vao->DrawArray(GL_LINES, 24);

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
