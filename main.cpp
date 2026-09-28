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

#include <cave-traversal-tool/UserSettings.h>

#include <cave-traversal-tool/Debug.h>
#include <cave-traversal-tool/ErrorCallbacks.h>
#include <cave-traversal-tool/FileIO.h>
#include <cave-traversal-tool/PFDWrapper.h>

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

struct GuiState
{
    bool display_project_tab       = true;
    bool display_user_settings_tab = true;
    bool display_debug_tab         = false;
};

// ProjectData instance : owns CPU and GPU side data for dataset and file paths
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
};

static GuiState     _gui_state     = {};
static UserSettings _user_settings = {};
static ProjectData  _project_data{};

static bool    g_use_fixed_lod   = true;
static int32_t g_fixed_lod_index = 0;

//
static bool     g_trajectory_index_auto_play           = false;
static int32_t  g_trajectory_index_auto_play_increment = 1;
static uint32_t g_trajectory_index                     = 0;

// gizmo
static bool g_modify_current_pose_with_gizmo = false;

// lock viewport 0 camera target to current trajectory pose
static bool g_lock_viewport0_target_to_trajectory = false;

static inline void snap_camera_target_to_trajectory(Camera& cam, const glm::vec3& pose_pos)
{
    const glm::vec3 offset = cam.position - cam.target;
    cam.target             = pose_pos;
    cam.position           = pose_pos + offset;
}

template <typename T>
static size_t std_vector_size(const std::vector<T>& vector)
{
    return vector.size() * sizeof(T);
}

static bool rebuild_trajectory_mat33_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_point = opengl_vertex_array_get_vertex_layout<Point>();

    if (_project_data.trajectory_positions.empty() || _project_data.trajectory_orientations_mat33.empty())
    {
        spdlog::error("Refusing to rebuild trajectory OpenGL data : no trajectory points loaded");
        return false;
    }

    g_trajectory_index = 0;

    if (_project_data.trajectory_positions_vao)
    {
        spdlog::debug("Deleting old trajectory positions VAO : {}", _project_data.trajectory_positions_vao->GetID());
        delete _project_data.trajectory_positions_vao;
    }

    if (_project_data.trajectory_positions_vbo)
    {
        spdlog::debug("Deleting old trajectory positions VBO : {}", _project_data.trajectory_positions_vbo->GetID());
        delete _project_data.trajectory_positions_vbo;
    }

    _project_data.trajectory_positions_vbo = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(_project_data.trajectory_positions), _project_data.trajectory_positions.data());
    _project_data.trajectory_positions_vao = new VertexArray(_project_data.trajectory_positions_vbo, false, nullptr, false, layout_point);

    spdlog::debug("Created VAO [{}] and VBO [{}]", _project_data.trajectory_positions_vao->GetID(), _project_data.trajectory_positions_vbo->GetID());

    return true;
}

static bool rebuild_stretcher_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_color_point = opengl_vertex_array_get_vertex_layout<ColorPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point       = opengl_vertex_array_get_vertex_layout<Point>();

    if (_project_data.stretcher_vertices.empty() || _project_data.stretcher_indices.empty())
    {
        spdlog::error("Refusing to rebuild stretcher OpenGL data : no stretcher data loaded");
        return false;
    }

    if (_project_data.stretcher_aabb_vao)
    {
        spdlog::debug("Deleting old stretcher AABB positions VAO : {}", _project_data.stretcher_aabb_vao->GetID());
        delete _project_data.stretcher_aabb_vao;
    }

    if (_project_data.stretcher_aabb_vbo)
    {
        spdlog::debug("Deleting old stretcher AABB positions VBO : {}", _project_data.stretcher_aabb_vbo->GetID());
        delete _project_data.stretcher_aabb_vbo;
    }

    if (_project_data.stretcher_vao)
    {
        spdlog::debug("Deleting old stretcher positions VAO : {}", _project_data.stretcher_vao->GetID());
        delete _project_data.stretcher_vao;
    }

    if (_project_data.stretcher_vbo)
    {
        spdlog::debug("Deleting old stretcher positions VBO : {}", _project_data.stretcher_vbo->GetID());
        delete _project_data.stretcher_vbo;
    }

    if (_project_data.stretcher_index_buffer)
    {
        spdlog::debug("Deleting old stretcher index IBO : {}", _project_data.stretcher_index_buffer->GetID());
        delete _project_data.stretcher_index_buffer;
    }

    _project_data.stretcher_aabb.min = _project_data.stretcher_vertices[0].position;
    _project_data.stretcher_aabb.max = _project_data.stretcher_vertices[0].position;

    for (const auto& v : _project_data.stretcher_vertices)
    {
        const glm::vec3& p = v.position;

        _project_data.stretcher_aabb.min = glm::min(_project_data.stretcher_aabb.min, p);
        _project_data.stretcher_aabb.max = glm::max(_project_data.stretcher_aabb.max, p);
    }

    std::vector<Point> line_vertices{
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.min.z}},

        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.max.z}},

        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.min.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.max.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.max.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.min.z}},
        {{_project_data.stretcher_aabb.min.x, _project_data.stretcher_aabb.max.y, _project_data.stretcher_aabb.max.z}}};

    _project_data.stretcher_aabb_vbo = new Buffer(GL_NONE, std_vector_size(line_vertices), line_vertices.data());
    _project_data.stretcher_aabb_vao = new VertexArray(_project_data.stretcher_aabb_vbo, false, nullptr, false, layout_point);

    _project_data.stretcher_vbo          = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(_project_data.stretcher_vertices), _project_data.stretcher_vertices.data());
    _project_data.stretcher_index_buffer = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(_project_data.stretcher_indices), _project_data.stretcher_indices.data());

    _project_data.stretcher_vao = new VertexArray(_project_data.stretcher_vbo, false, _project_data.stretcher_index_buffer, false, layout_color_point);

    spdlog::debug("Created VAO [{}], VBO [{}] and IBO [{}]", _project_data.stretcher_vao->GetID(), _project_data.stretcher_vbo->GetID(), _project_data.stretcher_index_buffer->GetID());

    return true;
}

static bool rebuild_cave_opengl_data()
{
    const std::vector<VertexBufferAttributeLayout> layout_point_intensity = opengl_vertex_array_get_vertex_layout<PointIntensity>();
    const std::vector<VertexBufferAttributeLayout> layout_point           = opengl_vertex_array_get_vertex_layout<Point>();

    if (_project_data.cave_vertices.empty())
    {
        spdlog::error("Refusing to rebuild cave OpenGL data : no cave points loaded");
        return false;
    }

    // Clear old data
    for (auto& [ID, bucket] : _project_data.buckets)
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

    _project_data.buckets.clear();

    _project_data.intensity_min = _project_data.cave_vertices.front().intensity;
    _project_data.intensity_max = _project_data.intensity_min;

    _project_data.cave_aabb.min = _project_data.cave_vertices.front().position;
    _project_data.cave_aabb.max = _project_data.cave_aabb.min;

    for (const auto& p : _project_data.cave_vertices)
    {
        _project_data.intensity_min = std::min(_project_data.intensity_min, p.intensity);
        _project_data.intensity_max = std::max(_project_data.intensity_max, p.intensity);

        _project_data.cave_aabb.min = glm::min(_project_data.cave_aabb.min, p.position);
        _project_data.cave_aabb.max = glm::max(_project_data.cave_aabb.max, p.position);
    }

    bucketize_point_cloud(_project_data.cave_vertices, _project_data.buckets,
                          _user_settings.io.map_load_extent,
                          _user_settings.io.map_load_decimation_factor,
                          _user_settings.io.map_load_decimation_levels,
                          _user_settings.io.map_load_minimum_first_level_points,
                          _user_settings.io.map_load_use_center_extent);

    for (auto& [ID, bucket] : _project_data.buckets)
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

    for (auto& [ID, bucket] : _project_data.buckets)
    {
        size_t bucket_lod_count = 0;
        for (PointCloudLOD* lod = bucket.lods; lod; lod = lod->next)
        {
            ++bucket_lod_count;
        }

        _project_data.max_lod_count = std::max(_project_data.max_lod_count, bucket_lod_count);
    }

    if (_project_data.buckets.empty())
    {
        spdlog::error("Bucketization produced no buckets from {} cave points", _project_data.cave_vertices.size());
        return false;
    }

    return true;
}

static inline void load_trajectory()
{
    std::string filename;

    if (PFDOpenFile("Open CSV file", "CSV Files (.csv)", "*.csv", filename))
    {
        if (!load_trajectory_csv(filename, _project_data.trajectory_positions, _project_data.trajectory_orientations_mat33, _user_settings.io.trajectory_load_every_nth))
        {
            spdlog::error("Failed to load trajectory CSV : {}", filename);
            return;
        }

        if (!rebuild_trajectory_mat33_opengl_data())
        {
            spdlog::error("Failed to rebuild trajectory OpenGL data for : {}", filename);
            return;
        }

        _project_data.trajectory_path = filename;
    }
}

// Move trajectory index by +- given amount of meters along the trajectory (if possible)
static void move_trajectory_index_by_distance(const std::vector<Point>& trajectory, uint32_t& index, const float amount)
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

static inline void load_object()
{
    std::string filename;

    if (PFDOpenFile("Open PLY file", "PLY Files (.ply)", "*.ply", filename))
    {
        if (!load_stretcher_ply(filename, _project_data.stretcher_vertices, _project_data.stretcher_indices))
        {
            spdlog::error("Failed to load stretcher PLY : {}", filename);
            return;
        }

        if (!rebuild_stretcher_opengl_data())
        {
            spdlog::error("Failed to rebuild stretcher OpenGL data for : {}", filename);
            return;
        }

        _project_data.object_path = filename;
    }
}

static inline void load_environment()
{
    std::string filename;

    if (PFDOpenFile("Open LAZ file", "LAZ Files (*.laz *.las)", "*.laz *.las", filename))
    {
        if (!load_cave_laz(filename, _project_data.cave_vertices))
        {
            spdlog::error("Failed to load environment LAZ : {}", filename);
            return;
        }

        if (!rebuild_cave_opengl_data())
        {
            spdlog::error("Failed to rebuild cave OpenGL data for : {}", filename);
            return;
        }

        _project_data.environment_path = filename;
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

    MultiViewContext ctx{};
    ctx.cameras[0].position = glm::vec3(10.0f, 10.0f, 10.0f);
    ctx.cameras[1].position = glm::vec3(-10.0f, 10.0f, 10.0f);
    ctx.cameras[2].position = glm::vec3(10.0f, -10.0f, 10.0f);
    ctx.cameras[3].position = glm::vec3(-10.0f, -10.0f, 10.0f);
    for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
    {
        Viewport vp               = ctx.viewport_for(i);
        ctx.cameras[i].viewport_w = static_cast<float>(vp.w);
        ctx.cameras[i].viewport_h = static_cast<float>(vp.h);
    }
    glfwSetWindowUserPointer(window, &ctx);

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
    Program* point_cloud_color_map_program  = make_program(GetProgramShaderSources_PointCloudColorMap());
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

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        int32_t width  = 0;
        int32_t height = 0;
        glfwGetWindowSize(window, &width, &height);
        ctx.window_width  = width;
        ctx.window_height = height;

        for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
        {
            Viewport vp               = ctx.viewport_for(i);
            ctx.cameras[i].viewport_w = static_cast<float>(vp.w);
            ctx.cameras[i].viewport_h = static_cast<float>(vp.h);
        }

        glViewport(0, 0, width, height);

        glClearColor(_user_settings.opengl.clear_color.x, _user_settings.opengl.clear_color.y, _user_settings.opengl.clear_color.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {
            static ImGuiDockNodeFlags dockspace_flags =
                ImGuiDockNodeFlags_PassthruCentralNode;

            static ImGuiWindowFlags window_flags =
                ImGuiWindowFlags_MenuBar |
                ImGuiWindowFlags_NoDocking |
                ImGuiWindowFlags_NoTitleBar |
                ImGuiWindowFlags_NoCollapse |
                ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoBringToFrontOnFocus |
                ImGuiWindowFlags_NoNavFocus |
                ImGuiWindowFlags_NoBackground;

            const ImGuiViewport* viewport = ImGui::GetMainViewport();

            ImGui::SetNextWindowPos(viewport->WorkPos);
            ImGui::SetNextWindowSize(viewport->WorkSize);
            ImGui::SetNextWindowViewport(viewport->ID);

            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

            ImGui::Begin("DockSpace Window", nullptr, window_flags);

            ImGui::PopStyleVar(3);

            ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
            ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

            if (ImGui::BeginMenuBar())
            {
                if (ImGui::BeginMenu("Display tabs"))
                {
                    ImGui::MenuItem("Display project tab", nullptr, &_gui_state.display_project_tab);
                    ImGui::MenuItem("Display user setting tab", nullptr, &_gui_state.display_user_settings_tab);
                    ImGui::MenuItem("Display debug tab", nullptr, &_gui_state.display_debug_tab);

                    ImGui::EndMenu();
                }

                ImGui::EndMenuBar();
            }
            ImGui::End();
        }

        {
            if (_gui_state.display_project_tab)
            {
                if (ImGui::Begin("Application", &_gui_state.display_project_tab))
                {
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

                        for (int i = 1; i <= 3; ++i)
                        {
                            if (i >= static_cast<int>(ctx.active_count))
                            {
                                continue;
                            }

                            ImGui::Separator();
                            ImGui::Text("Viewport %d", i + 1);

                            int mode = static_cast<int>(ctx.camera_modes[i]);
                            if (ImGui::Combo(("##camera_mode_" + std::to_string(i)).c_str(), &mode, camera_mode_names, 13))
                            {
                                CameraMode old_mode = ctx.camera_modes[i];
                                ctx.camera_modes[i] = static_cast<CameraMode>(mode);
                                if (ctx.camera_modes[i] != CameraMode::FREE_ORBIT && old_mode != ctx.camera_modes[i])
                                {
                                    ctx.cameras[i].near_plane = std::max(0.01f, ctx.view_axis_distance[i] - 1.5f);
                                    ctx.cameras[i].far_plane  = ctx.view_axis_distance[i] + 1.5f;
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

                            if (ctx.camera_modes[i] != CameraMode::FREE_ORBIT)
                            {
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

                        ImGui::TreePop();
                    }

                    ImGui::Separator();
                    if (ImGui::TreeNode("File input / output"))
                    {
                        if (ImGui::Button("Load trajectory", ImVec2(200.0f, 0.0f)))
                        {
                            load_trajectory();
                        }
                        ImGui::Text("%s", _project_data.trajectory_path.empty() ? "(none)" : _project_data.trajectory_path.c_str());

                        ImGui::Separator();

                        if (ImGui::Button("Load object", ImVec2(200.0f, 0.0f)))
                        {
                            load_object();
                        }
                        ImGui::Text("%s", _project_data.object_path.empty() ? "(none)" : _project_data.object_path.c_str());

                        ImGui::Separator();

                        if (ImGui::Button("Load environment", ImVec2(200.0f, 0.0f)))
                        {
                            load_environment();
                        }
                        ImGui::Text("%s", _project_data.environment_path.empty() ? "(none)" : _project_data.environment_path.c_str());

                        ImGui::TreePop();
                    }

                    ImGui::Separator();
                    if (ImGui::TreeNode("Level of Detail (LOD)"))
                    {
                        ImGui::Checkbox("g_use_fixed_lod", &g_use_fixed_lod);

                        const int32_t max_lod_index = _project_data.max_lod_count > 0 ? static_cast<int32_t>(_project_data.max_lod_count - 1) : 0;
                        ImGui::BeginDisabled(!g_use_fixed_lod);
                        ImGui::SliderInt("g_fixed_lod_index", &g_fixed_lod_index, 0, max_lod_index);
                        ImGui::EndDisabled();
                        ImGui::TreePop();
                    }

                    ImGui::Separator();
                    if (ImGui::TreeNode("Trajectory"))
                    {
                        if (_project_data.trajectory_orientations_mat33.size())
                        {
                            const uint32_t zero                  = 0U;
                            const uint32_t max_orientation_index = static_cast<uint32_t>(_project_data.trajectory_orientations_mat33.size()) - 1U;

                            ImGui::Text("Trajectory : %zu / %zu = %.2f%", static_cast<size_t>(g_trajectory_index), max_orientation_index, static_cast<float>(g_trajectory_index) / static_cast<float>(max_orientation_index) * 100.0f);
                            ImGui::Checkbox("g_trajectory_index_auto_play", &g_trajectory_index_auto_play);
                            ImGui::DragInt("g_trajectory_index_auto_play_increment", &g_trajectory_index_auto_play_increment, 1.0f, 1, INT32_MAX);
                            ImGui::DragScalar("g_trajectory_index", ImGuiDataType_U32, &g_trajectory_index, 1.0f, &zero, &max_orientation_index);

                            // Trajectory traversal buttons : move +- given meters along the trajectory
                            if (ImGui::Button("- 1 m"))
                            {
                                move_trajectory_index_by_distance(_project_data.trajectory_positions, g_trajectory_index, -1.0f);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("- 5 m"))
                            {
                                move_trajectory_index_by_distance(_project_data.trajectory_positions, g_trajectory_index, -5.0f);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("- 10 m"))
                            {
                                move_trajectory_index_by_distance(_project_data.trajectory_positions, g_trajectory_index, -10.0f);
                            }

                            if (ImGui::Button("+ 1 m"))
                            {
                                move_trajectory_index_by_distance(_project_data.trajectory_positions, g_trajectory_index, +1.0f);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("+ 5 m"))
                            {
                                move_trajectory_index_by_distance(_project_data.trajectory_positions, g_trajectory_index, +5.0f);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("+ 10 m"))
                            {
                                move_trajectory_index_by_distance(_project_data.trajectory_positions, g_trajectory_index, +10.0f);
                            }
                        }
                        else
                        {
                            ImGui::TextColored({1.0f, 0.0f, 0.0f, 1.0f}, "Can not set index of trajectory pose - load trajectory!");
                        }

                        ImGui::TreePop();
                    }

                    ImGui::Checkbox("g_modify_current_pose_with_gizmo", &g_modify_current_pose_with_gizmo);

                    ImGui::Separator();
                    ImGui::Checkbox("Lock target to trajectory", &g_lock_viewport0_target_to_trajectory);
                    ImGui::SameLine();
                    if (ImGui::Button("Snap"))
                    {
                        if (_project_data.trajectory_positions.size())
                        {
                            snap_camera_target_to_trajectory(ctx.cameras[0], _project_data.trajectory_positions[g_trajectory_index].position);
                        }
                    }
                    if (g_lock_viewport0_target_to_trajectory)
                    {
                        ImGui::TextDisabled("locked - viewport 0 target follows current trajectory pose each frame");
                    }
                }
                ImGui::End();
            }

            if (_gui_state.display_user_settings_tab)
            {
                UserSettingsImGUI(_user_settings, _gui_state.display_user_settings_tab);
            }

            if (_gui_state.display_debug_tab)
            {
                DebugImGUI(_project_data.buckets, _gui_state.display_debug_tab);
            }
        }

        glm::vec3 stretcher_position    = glm::vec3(0.0f);
        glm::mat3 stretcher_orientation = glm::mat3(1.0f);

        if (g_trajectory_index_auto_play && !_project_data.trajectory_orientations_mat33.empty())
        {
            const size_t last = _project_data.trajectory_orientations_mat33.size() - 1;

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

        if (_project_data.trajectory_positions.size() && _project_data.trajectory_orientations_mat33.size())
        {
            const auto& trajectory_point      = _project_data.trajectory_positions[g_trajectory_index];
            const auto& trajectoryorientation = _project_data.trajectory_orientations_mat33[g_trajectory_index];

            stretcher_position    = trajectory_point.position;
            stretcher_orientation = trajectoryorientation.orientation;
        }

        glm::mat4 stretcher_pose = glm::translate(glm::mat4(1.0f), stretcher_position) * glm::mat4(stretcher_orientation);

        const OBB  stretcher_obb               = aabb_to_obb(_project_data.stretcher_aabb, stretcher_pose);
        const auto in_obb_ids_in_obb_proximity = find_buckets_in_obb(_project_data.buckets, stretcher_obb, _user_settings.collision.radious);

        if (g_modify_current_pose_with_gizmo || glfwGetKey(window, GLFW_KEY_G))
        {
            glm::vec3 stretcher_position    = glm::vec3(0.0f);
            glm::mat3 stretcher_orientation = glm::mat3(1.0f);

            if (_project_data.trajectory_positions.size() && _project_data.trajectory_orientations_mat33.size())
            {
                const auto& trajectory_point      = _project_data.trajectory_positions[g_trajectory_index];
                const auto& trajectoryorientation = _project_data.trajectory_orientations_mat33[g_trajectory_index];

                stretcher_position    = trajectory_point.position;
                stretcher_orientation = trajectoryorientation.orientation;
            }

            glm::mat4 stretcher_pose = glm::translate(glm::mat4(1.0f), stretcher_position) * glm::mat4(stretcher_orientation);

            Viewport vp0    = ctx.viewport_for(0);
            float    rect_x = static_cast<float>(vp0.x);
            float    rect_y = static_cast<float>(height - vp0.y - vp0.h);
            float    rect_w = static_cast<float>(vp0.w);
            float    rect_h = static_cast<float>(vp0.h);

            glm::mat4 gizmo_proj = glm::perspectiveFov(glm::radians(55.0f), rect_w, rect_h, ctx.cameras[0].near_plane, ctx.cameras[0].far_plane);
            glm::mat4 gizmo_view = ctx.cameras[0].get_view();

            ImGuizmo::BeginFrame();
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist(ImGui::GetBackgroundDrawList());
            ImGuizmo::SetRect(rect_x, rect_y, rect_w, rect_h);

            float objectMatrix[16] =
                {1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1};

            std::memcpy(objectMatrix, glm::value_ptr(stretcher_pose), sizeof(glm::mat4));

            ImGuizmo::Manipulate(
                glm::value_ptr(gizmo_view),
                glm::value_ptr(gizmo_proj),
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

            if (_project_data.trajectory_positions.size() && _project_data.trajectory_orientations_mat33.size())
            {
                _project_data.trajectory_positions[g_trajectory_index].position             = position;
                _project_data.trajectory_orientations_mat33[g_trajectory_index].orientation = rotation;

                glNamedBufferSubData(_project_data.trajectory_positions_vbo->GetID(), sizeof(glm::vec3) * g_trajectory_index, sizeof(glm::vec3), &_project_data.trajectory_positions[g_trajectory_index].position);
            }
        }
        // VIEWPORT DIVIDER LINES
        {
            int vp_count = static_cast<int>(ctx.active_count);
            if (vp_count >= 2)
            {
                ImDrawList* dl  = ImGui::GetBackgroundDrawList();
                ImU32       col = IM_COL32(180, 180, 180, 200);

                // Vertical center line for modes 2 and 4
                dl->AddLine(
                    ImVec2(static_cast<float>(width) * 0.5f, 0.0f),
                    ImVec2(static_cast<float>(width) * 0.5f, static_cast<float>(height)),
                    col,
                    1.0f);

                // Horizontal center line for mode 4 only
                if (vp_count == 4)
                {
                    dl->AddLine(
                        ImVec2(0.0f, static_cast<float>(height) * 0.5f),
                        ImVec2(static_cast<float>(width), static_cast<float>(height) * 0.5f),
                        col,
                        1.0f);
                }
            }
        }

        ImGui::Render();

        // PICKING POINT CLOUD
        {
            static bool prev_mouse_pressed = false;

            bool ctrl_pressed  = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);
            bool alt_pressed   = (glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS);
            bool mouse_pressed = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);

            // Trigger once when mouse goes from released -> pressed, while Ctrl (bucket pick) or Alt (trajectory pick) is held
            bool new_click     = mouse_pressed && !prev_mouse_pressed && (ctrl_pressed || alt_pressed);
            prev_mouse_pressed = mouse_pressed;

            if (new_click && !ImGui::GetIO().WantCaptureMouse)
            {
                double mouse_x, mouse_y;
                glfwGetCursorPos(window, &mouse_x, &mouse_y);

                int pick_idx = ctx.camera_index_at(mouse_x, mouse_y);
                if (pick_idx >= 0)
                {
                    Camera&  pick_cam = ctx.cameras[pick_idx];
                    Viewport vp       = ctx.viewport_for(pick_idx);

                    double local_x    = mouse_x - vp.x;
                    double local_gl_y = (static_cast<double>(height) - mouse_y) - vp.y;

                    float x_ndc = (2.0f * static_cast<float>(local_x) / static_cast<float>(vp.w)) - 1.0f;
                    float y_ndc = (2.0f * static_cast<float>(local_gl_y) / static_cast<float>(vp.h)) - 1.0f;

                    glm::mat4 pick_projection = glm::perspectiveFov(glm::radians(55.0f), static_cast<float>(vp.w), static_cast<float>(vp.h), pick_cam.near_plane, pick_cam.far_plane);
                    glm::mat4 pick_view       = pick_cam.get_view();

                    glm::vec4 ray_clip(x_ndc, y_ndc, -1.0f, 1.0f);
                    glm::vec4 ray_eye = glm::inverse(pick_projection) * ray_clip;
                    ray_eye.z         = -1.0f;
                    ray_eye.w         = 0.0f;

                    glm::vec3 ray_dir        = glm::normalize(glm::vec3(glm::inverse(pick_view) * ray_eye));
                    glm::vec3 camera_pos     = pick_cam.position;
                    glm::vec3 camera_forward = glm::normalize(pick_cam.target - pick_cam.position);

                    if (alt_pressed)
                    {
                        // Pick closest trajectory point to the cast ray (within 0.25 m of the ray)
                        const float TRAJECTORY_PICK_RADIUS = 0.25f;

                        uint32_t best_index = 0;
                        float    best_dist  = TRAJECTORY_PICK_RADIUS;
                        float    best_t     = std::numeric_limits<float>::max();
                        bool     found      = false;

                        for (size_t i = 0; i < _project_data.trajectory_positions.size(); ++i)
                        {
                            const glm::vec3 to_point = _project_data.trajectory_positions[i].position - camera_pos;
                            const float     t        = glm::dot(to_point, ray_dir);

                            if (t <= 0.0f)
                            {
                                continue;
                            }

                            const float d = glm::length(to_point - t * ray_dir);

                            if (d > TRAJECTORY_PICK_RADIUS)
                            {
                                continue;
                            }

                            // Prefer smallest distance to ray, then the point closest to the camera
                            if (!found || d < best_dist - 1e-4f || (d < best_dist + 1e-4f && t < best_t))
                            {
                                found      = true;
                                best_dist  = d;
                                best_t     = t;
                                best_index = static_cast<uint32_t>(i);
                            }
                        }

                        if (found)
                        {
                            spdlog::info("Trajectory pick in viewport {} : index = [{}] (distance to ray {:.3f} m)", pick_idx, best_index, best_dist);
                            g_trajectory_index = best_index;
                        }
                        else
                        {
                            spdlog::warn("Trajectory picking missed ... (no trajectory point within {:.2f} m of ray)", TRAJECTORY_PICK_RADIUS);
                        }
                    }
                    else if (ctrl_pressed)
                    {
                        PointCloudRecord* picked_record = nullptr;
                        glm::ivec3        picked_id{};
                        float             closest_dist = std::numeric_limits<float>::max();

                        const float PICK_RADIUS = 0.1f;

                        for (auto& [ID, bucket] : _project_data.buckets)
                        {
                            glm::vec3 center    = 0.5f * (bucket.aabb.min + bucket.aabb.max);
                            glm::vec3 to_center = center - camera_pos;

                            if (glm::dot(to_center, camera_forward) <= 0.0f)
                            {
                                continue;
                            }

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

                        if (picked_record)
                        {
                            spdlog::info("Picking hit in viewport {} : ID = [{} {} {}]", pick_idx, picked_id.x, picked_id.y, picked_id.z);

                            glm::vec3 center  = picked_record->aabb.min + 0.5f * (picked_record->aabb.max - picked_record->aabb.min);
                            glm::vec3 offset  = pick_cam.position - pick_cam.target;
                            pick_cam.target   = center;
                            pick_cam.position = pick_cam.target + offset;
                        }
                        else
                        {
                            spdlog::warn("Picking missed ...");
                        }
                    }
                }
            }
        }

        auto draw_scene = [&](const uint32_t viewport_index, const Viewport& vp, Camera& cam)
        {
            glViewport(vp.x, vp.y, vp.w, vp.h);

            glm::mat4 projection = glm::perspectiveFov(glm::radians(55.0f), static_cast<float>(vp.w), static_cast<float>(vp.h), cam.near_plane, cam.far_plane);
            glm::mat4 view       = cam.get_view();
            glm::mat4 MVP        = projection * view;

            std::array<glm::vec4, 6> frustum{};
            compute_camera_frustum_planes(view, projection, frustum);

            // ORIGIN
            if (_user_settings.origin.draw_enable)
            {
                glLineWidth(_user_settings.origin.width);
                origin_program->Bind();
                origin_program->PushUniform16F32("u_MVP", MVP);
                origin_program->PushUniform1F32("u_Scale", _user_settings.origin.scale);
                origin_vao->Bind();
                origin_vao->DrawArray(GL_LINES, 6);
                glLineWidth(1.0f);
            }

            // CAMERA_TARGET
            if (_user_settings.target.draw_enable)
            {
                glLineWidth(_user_settings.target.width);
                camera_target_program->Bind();
                camera_target_program->PushUniform16F32("u_MVP", MVP);
                camera_target_program->PushUniform3F32("u_Translation", cam.target);
                camera_target_program->PushUniform1F32("u_Scale", _user_settings.target.scale);
                camera_target_program->PushUniform3F32("u_Color", _user_settings.target.color);
                target_vao->Bind();
                target_vao->DrawArray(GL_LINES, 6);
                glLineWidth(1.0f);
            }

            // TRAJECTORY
            if (_user_settings.trajectory.draw_enable && (_project_data.trajectory_positions.size() && _project_data.trajectory_orientations_mat33.size()))
            {
                glLineWidth(_user_settings.trajectory.width);
                trajectory_program->Bind();
                trajectory_program->PushUniform16F32("u_MVP", MVP);
                trajectory_program->PushUniform3F32("u_Color", _user_settings.trajectory.color);
                _project_data.trajectory_positions_vao->Bind();
                _project_data.trajectory_positions_vao->DrawArray(GL_LINE_STRIP, _project_data.trajectory_positions.size());
                glLineWidth(1.0f);
            }

            //  STRETCHER
            if (_user_settings.stretcher.draw_enable && (_project_data.stretcher_vertices.size() && _project_data.stretcher_indices.size()))
            {
                stretcher_program->Bind();
                stretcher_program->PushUniform16F32("u_MVP", MVP);
                stretcher_program->PushUniform16F32("u_Pose", stretcher_pose);

                _project_data.stretcher_vao->Bind();
                _project_data.stretcher_vao->DrawElements(GL_TRIANGLES, _project_data.stretcher_indices.size(), 1, 0);
            }

            //  STRETCHER BBOX
            if (_user_settings.stretcher.draw_enable_bbox && (_project_data.stretcher_vertices.size() && _project_data.stretcher_indices.size()))
            {
                glLineWidth(_user_settings.stretcher.bbox_width);

                bounding_box_stretcher_program->Bind();
                bounding_box_stretcher_program->PushUniform16F32("u_MVP", MVP);
                bounding_box_stretcher_program->PushUniform3F32("u_Color", _user_settings.stretcher.bbox_color);
                bounding_box_stretcher_program->PushUniform16F32("u_Pose", stretcher_pose);
                _project_data.stretcher_aabb_vao->Bind();
                _project_data.stretcher_aabb_vao->DrawArray(GL_LINES, 24);
                glLineWidth(1.0f);
            }

            const bool draw_any_cave_lod = _user_settings.point_cloud.draw_enable_pc_out ||
                                           _user_settings.point_cloud.draw_enable_pc_in_obb ||
                                           _user_settings.point_cloud.draw_enable_pc_in_obb_proximity;

            // POINT_CLOUD
            if (_user_settings.point_cloud.draw_enable_pc && _project_data.buckets.size() && draw_any_cave_lod)
            {
                glm::vec3 camera_pos = glm::vec3(glm::inverse(view)[3]);

                glPointSize(_user_settings.point_cloud.point_size);

                const bool use_color_map    = _user_settings.point_cloud.display_mode != PointCloudDisplayMode::Intensity;
                const bool use_position_map = _user_settings.point_cloud.display_mode == PointCloudDisplayMode::ColorMapPosition ||
                                              _user_settings.point_cloud.display_mode == PointCloudDisplayMode::ColorMapPositionTimesIntensity;

                Program* active_point_cloud_program = use_color_map ? point_cloud_color_map_program : point_cloud_program;
                active_point_cloud_program->Bind();
                active_point_cloud_program->PushUniform16F32("u_MVP", MVP);

                if (use_color_map)
                {
                    active_point_cloud_program->PushUniform1F32("u_IntensityMin", _project_data.intensity_min);
                    active_point_cloud_program->PushUniform1F32("u_IntensityInvRange", (_project_data.intensity_max > _project_data.intensity_min) ? 1.0f / (_project_data.intensity_max - _project_data.intensity_min) : 1.0f);
                    active_point_cloud_program->PushUniform1F32("u_UsePosition", use_position_map ? 1.0f : 0.0f);
                    active_point_cloud_program->PushUniform3F32("u_PositionMin", _project_data.cave_aabb.min);
                    const glm::vec3 position_extent = glm::max(_project_data.cave_aabb.max - _project_data.cave_aabb.min, glm::vec3(1e-6f));
                    active_point_cloud_program->PushUniform3F32("u_PositionInvRange", 1.0f / position_extent);
                    active_point_cloud_program->PushUniform1F32("u_MultiplyIntensity", _user_settings.point_cloud.display_mode == PointCloudDisplayMode::ColorMapTimesIntensity ||
                                                                                               _user_settings.point_cloud.display_mode == PointCloudDisplayMode::ColorMapPositionTimesIntensity
                                                                                           ? 1.0f
                                                                                           : 0.0f);
                }

                for (auto& [ID, bucket] : _project_data.buckets)
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

                    if (is_in_obb && _user_settings.point_cloud.draw_enable_pc_in_obb)
                    {
                        lod->vao->Bind();
                        lod->vao->DrawArray(GL_POINTS, lod->points.size());
                        continue;
                    }

                    if (is_on_obb_proximity && _user_settings.point_cloud.draw_enable_pc_in_obb_proximity)
                    {
                        lod->vao->Bind();
                        lod->vao->DrawArray(GL_POINTS, lod->points.size());
                        continue;
                    }

                    if (_user_settings.point_cloud.draw_enable_pc_out && !(is_in_obb || is_on_obb_proximity))
                    {
                        lod->vao->Bind();
                        lod->vao->DrawArray(GL_POINTS, lod->points.size());
                        continue;
                    }
                }

                glPointSize(1.0f);
            }

            const bool draw_any_cave_boxes = _user_settings.point_cloud.draw_enable_bbox_out ||
                                             _user_settings.point_cloud.draw_enable_bbox_in_obb ||
                                             _user_settings.point_cloud.draw_enable_bbox_in_obb_proximity;

            // POINT CLOUD BOXES
            if (_user_settings.point_cloud.draw_enable_bbox && _project_data.buckets.size() && (draw_any_cave_boxes))
            {
                glm::vec3 camera_pos = glm::vec3(glm::inverse(view)[3]);

                bounding_box_program->Bind();
                bounding_box_program->PushUniform16F32("u_MVP", MVP);

                for (auto& [ID, bucket] : _project_data.buckets)
                {
                    if (!record_in_camera_frustum(bucket, frustum))
                    {
                        continue;
                    }

                    const bool is_in_obb           = std::ranges::contains(in_obb_ids_in_obb_proximity.first, ID);
                    const bool is_on_obb_proximity = std::ranges::contains(in_obb_ids_in_obb_proximity.second, ID);

                    if (is_in_obb && _user_settings.point_cloud.draw_enable_bbox_in_obb)
                    {
                        glLineWidth(_user_settings.point_cloud.bbox_width_in_obb);
                        glm::vec3 red(1.0f, 0.0f, 0.0f);

                        bounding_box_program->PushUniform3F32("u_Color", red);

                        bucket.bbox_vao->Bind();
                        bucket.bbox_vao->DrawArray(GL_LINES, 24);

                        glLineWidth(1.0f);

                        continue;
                    }
                    else if (is_on_obb_proximity && _user_settings.point_cloud.draw_enable_bbox_in_obb_proximity)
                    {
                        glLineWidth(_user_settings.point_cloud.bbox_width_in_obb_proximity);
                        glm::vec3 blue(0.0f, 0.0f, 1.0f);

                        bounding_box_program->PushUniform3F32("u_Color", blue);

                        bucket.bbox_vao->Bind();
                        bucket.bbox_vao->DrawArray(GL_LINES, 24);

                        glLineWidth(1.0f);

                        continue;
                    }
                    else if (_user_settings.point_cloud.draw_enable_bbox_out && !(is_in_obb || is_on_obb_proximity))
                    {
                        glLineWidth(_user_settings.point_cloud.bbox_width);
                        glm::vec3 white(1.0f, 1.0f, 1.0f);

                        bounding_box_program->PushUniform3F32("u_Color", white);

                        bucket.bbox_vao->Bind();
                        bucket.bbox_vao->DrawArray(GL_LINES, 24);

                        glLineWidth(1.0f);

                        continue;
                    }
                }
            }
        };

        const int count = static_cast<int>(ctx.active_count);

        if (g_lock_viewport0_target_to_trajectory && _project_data.trajectory_positions.size())
        {
            snap_camera_target_to_trajectory(ctx.cameras[0], _project_data.trajectory_positions[g_trajectory_index].position);
        }

        for (int i = 0; i < count; ++i)
        {
            if (i >= 1 && ctx.camera_modes[i] != CameraMode::FREE_ORBIT)
            {
                update_locked_camera(ctx.cameras[i], ctx.camera_modes[i], ctx.view_axis_distance[i], stretcher_position, stretcher_orientation);
            }
            else if (ctx.cameras[i].up != glm::vec3(0.0f, 0.0f, 1.0f))
            {
                unlock_camera_to_free_orbit(ctx.cameras[i], ctx.view_axis_distance[i]);
            }
        }

        for (int i = 0; i < count; ++i)
        {
            draw_scene(i, ctx.viewport_for(i), ctx.cameras[i]);
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
