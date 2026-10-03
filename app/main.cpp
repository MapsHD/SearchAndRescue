// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// clang-format on

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <ImGuizmo.h>

#include <implot.h>

#include <Core/Camera.h>
#include <Core/Debug.h>
#include <Core/ErrorCallbacks.h>
#include <Core/OpenGL/Buffer.h>
#include <Core/OpenGL/Program.h>
#include <Core/OpenGL/VertexArray.h>
#include <Core/PFDWrapper.h>
#include <Core/Processing.h>
#include <Core/Project.h>
#include <Core/ProjectGui.h>
#include <Core/Shaders.h>
#include <Core/Structures.h>
#include <Core/UserSettings.h>

#include <HDM_SAR_ConfigureInfo.hpp>

#define CONCATENATE_STRING_2(x) #x
#define CONCATENATE_STRING(x) CONCATENATE_STRING_2(x)

#ifndef HDMAPPING_REARCH_AND_RESCUE_VERSION_MAJOR
    #define HDMAPPING_REARCH_AND_RESCUE_VERSION_MAJOR 0
#endif

#ifndef HDMAPPING_REARCH_AND_RESCUE_VERSION_MINOR
    #define HDMAPPING_REARCH_AND_RESCUE_VERSION_MINOR 0
#endif

#ifndef HDMAPPING_REARCH_AND_RESCUE_VERSION_PATCH
    #define HDMAPPING_REARCH_AND_RESCUE_VERSION_PATCH 0
#endif

#if HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1
static constexpr uint32_t DEFAULT_WINDOW_OPENGL_CONTEXT_MAJOR = 4;
static constexpr uint32_t DEFAULT_WINDOW_OPENGL_CONTEXT_MINOR = 1;
#else
static constexpr uint32_t DEFAULT_WINDOW_OPENGL_CONTEXT_MAJOR = 4;
static constexpr uint32_t DEFAULT_WINDOW_OPENGL_CONTEXT_MINOR = 6;
#endif

static constexpr uint32_t DEFAULT_WINDOW_WIDTH  = 1600;
static constexpr uint32_t DEFAULT_WINDOW_HEIGHT = 900;

static constexpr const char* WINDOW_TITLE =
    "HDMapping-SearchAndRescue : " CONCATENATE_STRING(HDMAPPING_REARCH_AND_RESCUE_VERSION_MAJOR) "." CONCATENATE_STRING(HDMAPPING_REARCH_AND_RESCUE_VERSION_MINOR) "." CONCATENATE_STRING(HDMAPPING_REARCH_AND_RESCUE_VERSION_PATCH) " - HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1 = " CONCATENATE_STRING(HDMAPPING_REARCH_AND_RESCUE_USE_OPENGL_4_1) " - HDMAPPING_SEARCH_AND_RESCUE_USE_GLSL_410 = " CONCATENATE_STRING(HDMAPPING_SEARCH_AND_RESCUE_USE_GLSL_410);

struct GuiState
{
    bool display_project_tab       = true;
    bool display_user_settings_tab = false;
    bool display_debug_tab         = false;
};

static GuiState     _gui_state     = {};
static UserSettings _user_settings = {};
static ProjectData  _project_data  = {};

// GLFW drop callback : route each dropped file to the appropriate loader based on its extension
static void drop_callback(GLFWwindow*, int count, const char** paths)
{
    for (int i = 0; i < count; ++i)
    {
        std::filesystem::path path(paths[i]);

        // Extension comparison is case-insensitive (.LAS == .las)
        std::string extension = path.extension().string();
        for (char& c : extension)
        {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }

        bool loaded = false;

        if (extension == ".csv")
        {
            spdlog::info("Dropped file [{}] : loading trajectory", paths[i]);
            loaded = load_trajectory(_project_data, path.string(), _user_settings.io.trajectory_load_every_nth);
        }
        else if (extension == ".ply")
        {
            spdlog::info("Dropped file [{}] : loading stretcher object", paths[i]);
            loaded = load_object(_project_data, path.string());
        }
        else if (extension == ".las" || extension == ".laz")
        {
            spdlog::info("Dropped file [{}] : loading environment", paths[i]);
            loaded = load_environment(_project_data, path.string(), _user_settings);
        }
        else
        {
            spdlog::warn("Dropped file [{}] : unsupported extension [{}], supported : .las .laz .csv .ply", paths[i], extension);
        }

        if (!loaded && (extension == ".csv" || extension == ".ply" || extension == ".las" || extension == ".laz"))
        {
            spdlog::error("Failed to load dropped file : {}", paths[i]);
        }
    }
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

    if (!glfwInit())
    {
        spdlog::critical("Failed to initialize GLFW!");
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, DEFAULT_WINDOW_OPENGL_CONTEXT_MAJOR);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, DEFAULT_WINDOW_OPENGL_CONTEXT_MINOR);

    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    glfwWindowHint(GLFW_CONTEXT_NO_ERROR, GLFW_FALSE);

    GLFWwindow* window = glfwCreateWindow(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT, WINDOW_TITLE, nullptr, nullptr);

    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetWindowSizeCallback(window, size_callback);
    glfwSetDropCallback(window, drop_callback);

    MultiViewContext& ctx   = _project_data.multi_view;
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
    glfwSetWindowUserPointer(window, &_project_data.multi_view);

    glfwMakeContextCurrent(window);

    glfwSwapInterval(1);

    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    // glEnable(GL_DEBUG_OUTPUT);
    // glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    // glDebugMessageCallback(ErrorCallback::OpenGL, nullptr);

    // glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    Program* origin_program                 = make_program(GetProgramShaderSources_Origin());
    Program* camera_target_program          = make_program(GetProgramShaderSources_CameraTarger());
    Program* point_cloud_program            = make_program(GetProgramShaderSources_PointCloud());
    Program* point_cloud_color_map_program  = make_program(GetProgramShaderSources_PointCloudColorMap());
    Program* trajectory_program             = make_program(GetProgramShaderSources_Trajectory());
    Program* trajectory_axes_program        = make_program(GetProgramShaderSources_TrajectoryOrientations());
    Program* stretcher_program              = make_program(GetProgramShaderSources_Stretcher());
    Program* bounding_box_program           = make_program(GetProgramShaderSources_BoundingBox());
    Program* bounding_box_stretcher_program = make_program(GetProgramShaderSources_BoundingBoxStretcher());
    Program* colored_line_program           = make_program(GetProgramShaderSources_ColoredLine());

    const std::vector<VertexBufferAttributeLayout> layout_color_point = opengl_vertex_array_get_vertex_layout<ColorPoint>();
    const std::vector<VertexBufferAttributeLayout> layout_point       = opengl_vertex_array_get_vertex_layout<Point>();
    const std::vector<VertexBufferAttributeLayout> layout_colored     = opengl_vertex_array_get_vertex_layout<ColoredVertex>();

    Buffer*      origin_buffer = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(origin), origin.data());
    VertexArray* origin_vao    = new VertexArray({{origin_buffer, false, layout_color_point, 0}}, nullptr, false);

    Buffer*      target_buffer = new Buffer(GL_DYNAMIC_STORAGE_BIT, std_vector_size(target), target.data());
    VertexArray* target_vao    = new VertexArray({{target_buffer, false, layout_point, 0}}, nullptr, false);

    // Collision points : pre-allocated GPU buffer, filled each frame with positions of first-LOD points inside the stretcher OBB
    _project_data.collision_points_vbo = new Buffer(GL_DYNAMIC_STORAGE_BIT, ProjectData::COLLISION_POINTS_CAPACITY * sizeof(Point), nullptr);
    _project_data.collision_points_vao = new VertexArray({{_project_data.collision_points_vbo, false, layout_point, 0}}, nullptr, false);

    // Measurement lines : pre-allocated GPU buffer, 2 coloured vertices per completed entry
    _project_data.measurement_line_vbo = new Buffer(GL_DYNAMIC_STORAGE_BIT, 2 * ProjectData::MEASUREMENT_LINE_CAPACITY * sizeof(ColoredVertex), nullptr);
    _project_data.measurement_line_vao = new VertexArray({{_project_data.measurement_line_vbo, false, layout_colored, 0}}, nullptr, false);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Input state : query keys and mouse buttons once per frame and derive bools
        const bool g_key   = (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS);
        const bool s_key   = (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
        const bool ctrl    = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);
        const bool alt     = (glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS);
        const bool shift   = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                             (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
        const bool mouse_l = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        const bool mouse_r = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

        static bool s_prev       = false;
        static bool mouse_l_prev = false;
        static bool mouse_r_prev = false;

        const bool s_key_clicked   = s_key && !s_prev;         // one-shot S press
        const bool new_left_click  = mouse_l && !mouse_l_prev; // released -> pressed
        const bool new_right_click = mouse_r && !mouse_r_prev; // released -> pressed

        // Shift + S : toggle continuous snap of viewport 0 camera target to the current trajectory pose
        static bool shift_s_prev    = false;
        const bool  shift_s_clicked = shift && s_key && !shift_s_prev;
        shift_s_prev                = shift && s_key;

        s_prev       = s_key;
        mouse_l_prev = mouse_l;
        mouse_r_prev = mouse_r;

        int32_t width  = 0;
        int32_t height = 0;
        glfwGetWindowSize(window, &width, &height);
        int32_t framebuffer_width  = 0;
        int32_t framebuffer_height = 0;
        glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

        if (width <= 0 || height <= 0 || framebuffer_width <= 0 || framebuffer_height <= 0)
        {
            glfwWaitEvents();
            continue;
        }

        ctx.window_width  = width;
        ctx.window_height = height;

        for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
        {
            Viewport vp               = ctx.viewport_for(i);
            ctx.cameras[i].viewport_w = static_cast<float>(vp.w);
            ctx.cameras[i].viewport_h = static_cast<float>(vp.h);
        }

        glViewport(0, 0, framebuffer_width, framebuffer_height);

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

                // Right-aligned info tooltips: Authors | Shortcuts | Configuration
                // Each label is separated by 20 px; the group sits 20 px from the right edge.
                {
                    static constexpr float k_gap    = 20.0f;
                    static constexpr float k_margin = 20.0f;

                    const float authors_width       = ImGui::CalcTextSize("Authors").x;
                    const float shortcuts_width     = ImGui::CalcTextSize("Shortcuts").x;
                    const float configuration_width = ImGui::CalcTextSize("Configuration").x;

                    const float total_width = authors_width + k_gap + shortcuts_width + k_gap + configuration_width + k_margin;
                    ImGui::SetCursorPosX(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - total_width);

                    ImGui::Text("Authors");
                    if (ImGui::BeginItemTooltip())
                    {
                        if (ImGui::BeginTable("##authors", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                        {
                            ImGui::TableSetupColumn("ID");
                            ImGui::TableSetupColumn("Name");
                            ImGui::TableSetupColumn("E-mail");
                            ImGui::TableSetupColumn("Role");
                            ImGui::TableHeadersRow();

                            auto author_row = [](const char* id, const char* name, const char* email, const char* role)
                            {
                                ImGui::TableNextRow();
                                ImGui::TableSetColumnIndex(0);
                                ImGui::TextUnformatted(id);
                                ImGui::TableSetColumnIndex(1);
                                ImGui::TextUnformatted(name);
                                ImGui::TableSetColumnIndex(2);
                                ImGui::TextUnformatted(email);
                                ImGui::TableSetColumnIndex(3);
                                ImGui::TextUnformatted(role);
                            };

                            author_row("1", "Michal Wlasiuk", "michal.mwa87@gmail.com", "Project development");
                            author_row("2", "Janusz Bedkowski", "januszbedkowski@gmail.com", "Project supervisor");

                            ImGui::EndTable();
                        }
                        ImGui::EndTooltip();
                    }

                    ImGui::SameLine(0.0f, k_gap);

                    ImGui::Text("Shortcuts");
                    if (ImGui::BeginItemTooltip())
                    {
                        auto shortcut_row = [](const char* id, const char* input, const char* description)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::TextUnformatted(id);
                            ImGui::TableSetColumnIndex(1);
                            ImGui::TextUnformatted(input);
                            ImGui::TableSetColumnIndex(2);
                            ImGui::TextUnformatted(description);
                        };

                        ImGui::TextUnformatted("Movement");
                        if (ImGui::BeginTable("##shortcuts_movement", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                        {
                            ImGui::TableSetupColumn("ID");
                            ImGui::TableSetupColumn("Input");
                            ImGui::TableSetupColumn("Description");
                            ImGui::TableHeadersRow();

                            shortcut_row("0", "LMB drag", "Orbit the camera around its target");
                            shortcut_row("1", "RMB drag", "Pan the camera");
                            shortcut_row("2", "Scroll", "Zoom (hold Shift for faster zoom)");
                            shortcut_row("3", "Ctrl + LMB", "Pick point cloud bucket : camera target moves to its center");
                            shortcut_row("4", "Ctrl + RMB", "Pick bucket and focus camera target on closest point to the ray");
                            shortcut_row("5", "Alt + LMB", "Pick trajectory point : sets the current trajectory index");
                            shortcut_row("6", "Alt + RMB", "Snap camera target to trajectory point (leaves stretcher)");

                            ImGui::EndTable();
                        }

                        ImGui::Spacing();

                        ImGui::TextUnformatted("Processing");
                        if (ImGui::BeginTable("##shortcuts_processing", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                        {
                            ImGui::TableSetupColumn("ID");
                            ImGui::TableSetupColumn("Input");
                            ImGui::TableSetupColumn("Description");
                            ImGui::TableHeadersRow();

                            shortcut_row("0", "G", "Show stretcher gizmo : move / rotate current trajectory pose");
                            shortcut_row("1", "S", "Snap viewport 1 camera target to current trajectory pose");
                            shortcut_row("2", "Shift + S", "Toggle continuous snap of viewport 1 camera target to trajectory pose");
                            shortcut_row("3", "Shift + LMB", "Measurement : pick start point, then pick end point to measure distance");

                            ImGui::EndTable();
                        }
                        ImGui::EndTooltip();
                    }

                    ImGui::SameLine(0.0f, k_gap);

                    ImGui::Text("Configuration");
                    if (ImGui::BeginItemTooltip())
                    {
                        if (ImGui::BeginTable("##configuration", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                        {
                            ImGui::TableSetupColumn("ID");
                            ImGui::TableSetupColumn("Name");
                            ImGui::TableSetupColumn("Value");
                            ImGui::TableHeadersRow();

                            auto info_row = [](const char* id, const char* name, const char* velue)
                            {
                                ImGui::TableNextRow();
                                ImGui::TableSetColumnIndex(0);
                                ImGui::TextUnformatted(id);
                                ImGui::TableSetColumnIndex(1);
                                ImGui::TextUnformatted(name);
                                ImGui::TableSetColumnIndex(2);
                                ImGui::TextUnformatted(velue);
                            };

                            info_row("0", "HDM_SAR_CONFIGURE_BUILD_TYPES", HDM_SAR_CONFIGURE_BUILD_TYPES);
                            info_row("1", "HDM_SAR_CONFIGURE_IS_MULTICONFIG", HDM_SAR_CONFIGURE_IS_MULTICONFIG);
                            info_row("2", "HDM_SAR_CONFIGURE_SYSTEM", HDM_SAR_CONFIGURE_SYSTEM);
                            info_row("3", "HDM_SAR_CONFIGURE_SYSTEM_VERSION", HDM_SAR_CONFIGURE_SYSTEM_VERSION);
                            info_row("4", "HDM_SAR_CONFIGURE_CPU", HDM_SAR_CONFIGURE_CPU);
                            info_row("5", "HDM_SAR_CONFIGURE_COMPILER", HDM_SAR_CONFIGURE_COMPILER);
                            info_row("6", "HDM_SAR_CONFIGURE_COMPILER_VERSION", HDM_SAR_CONFIGURE_COMPILER_VERSION);
                            info_row("7", "HDM_SAR_CONFIGURE_COMPILER_PATH", HDM_SAR_CONFIGURE_COMPILER_PATH);
                            info_row("8", "HDM_SAR_CONFIGURE_CMAKE_VERSION", HDM_SAR_CONFIGURE_CMAKE_VERSION);
                            info_row("9", "HDM_SAR_CONFIGURE_GENERATOR", HDM_SAR_CONFIGURE_GENERATOR);
                            info_row("10", "HDM_SAR_CONFIGURE_PROJECT_NAME", HDM_SAR_CONFIGURE_PROJECT_NAME);
                            info_row("11", "HDM_SAR_CONFIGURE_PROJECT_VERSION", HDM_SAR_CONFIGURE_PROJECT_VERSION);
                            info_row("12", "HDM_SAR_CONFIGURE_CXX_STANDARD", HDM_SAR_CONFIGURE_CXX_STANDARD);
                            info_row("13", "HDM_SAR_CONFIGURE_TIMESTAMP", HDM_SAR_CONFIGURE_TIMESTAMP);
                            info_row("14", "HDM_SAR_CONFIGURE_GIT_HASH", HDM_SAR_CONFIGURE_GIT_HASH);
                            info_row("15", "HDM_SAR_CONFIGURE_GIT_BRANCH", HDM_SAR_CONFIGURE_GIT_BRANCH);
                            info_row("16", "HDM_SAR_CONFIGURE_IS_64_BIT", HDM_SAR_CONFIGURE_IS_64_BIT);
                            info_row("17", "HDM_SAR_CONFIGURE_CXX_FLAGS", HDM_SAR_CONFIGURE_CXX_FLAGS);
                            info_row("18", "HDM_SAR_CONFIGURE_CXX_FLAGS_DEBUG", HDM_SAR_CONFIGURE_CXX_FLAGS_DEBUG);
                            info_row("19", "HDM_SAR_CONFIGURE_CXX_FLAGS_RELEASE", HDM_SAR_CONFIGURE_CXX_FLAGS_RELEASE);
                            info_row("20", "HDM_SAR_CONFIGURE_EXE_LINKER_FLAGS", HDM_SAR_CONFIGURE_EXE_LINKER_FLAGS);
                            info_row("21", "HDM_SAR_CONFIGURE_HOST_SYSTEM", HDM_SAR_CONFIGURE_HOST_SYSTEM);
                            info_row("22", "HDM_SAR_CONFIGURE_HOST_SYSTEM_VERSION", HDM_SAR_CONFIGURE_HOST_SYSTEM_VERSION);
                            info_row("23", "HDM_SAR_CONFIGURE_HOST_CPU", HDM_SAR_CONFIGURE_HOST_CPU);

                            ImGui::EndTable();
                        }
                        ImGui::EndTooltip();
                    }
                }

                ImGui::EndMenuBar();
            }
            ImGui::End();
        }

        {
            if (_gui_state.display_project_tab)
            {
                ProjectDataImGUI(_project_data, _user_settings, _gui_state.display_project_tab);
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

        if (_project_data.trajectory_index_auto_play && !_project_data.trajectory_orientations_mat33.empty())
        {
            const size_t last = _project_data.trajectory_orientations_mat33.size() - 1;

            if (_project_data.trajectory_index + _project_data.trajectory_index_auto_play_increment >= last)
            {
                _project_data.trajectory_index           = last;
                _project_data.trajectory_index_auto_play = false;
            }
            else
            {
                _project_data.trajectory_index += _project_data.trajectory_index_auto_play_increment;
            }
        }

        if (_project_data.trajectory_positions.size() && _project_data.trajectory_orientations_mat33.size())
        {
            const auto& trajectory_point      = _project_data.trajectory_positions[_project_data.trajectory_index];
            const auto& trajectoryorientation = _project_data.trajectory_orientations_mat33[_project_data.trajectory_index];

            stretcher_position    = trajectory_point.position;
            stretcher_orientation = trajectoryorientation.orientation;
        }

        glm::mat4 stretcher_pose = glm::translate(glm::mat4(1.0f), stretcher_position) * glm::mat4(stretcher_orientation);

        const OBB  stretcher_obb               = aabb_to_obb(_project_data.stretcher_aabb, stretcher_pose);
        const auto in_obb_ids_in_obb_proximity = find_buckets_in_obb(_project_data.buckets, stretcher_obb, _user_settings.collision.radious);

        // Collect first-LOD points of colliding buckets that are inside the stretcher OBB, upload them for rendering
        size_t collision_point_count = 0;
        {
            // TODO(m.wlasiuk) : move this to project + limit amount based on point cloud statistics
            static std::vector<Point> collision_points{};
            collision_points.clear();

            for (const glm::ivec3& id : in_obb_ids_in_obb_proximity.first)
            {
                auto bucket_it = _project_data.buckets.find(id);
                if (bucket_it == _project_data.buckets.end())
                {
                    continue;
                }

                PointCloudLOD* first_lod = get_lod_at_index(&bucket_it->second, 0);
                if (!first_lod)
                {
                    continue;
                }

                for (const PointIntensity& p : first_lod->points)
                {
                    if (point_in_obb(p.position, stretcher_obb))
                    {
                        collision_points.push_back({p.position});

                        // TODO(m.wlasiuk) : limit amount based on point cloud statistics
                        if (collision_points.size() >= ProjectData::COLLISION_POINTS_CAPACITY)
                        {
                            spdlog::warn("Collision point buffer full : {} points, ignoring the rest", ProjectData::COLLISION_POINTS_CAPACITY);
                            break;
                        }
                    }
                }

                if (collision_points.size() >= ProjectData::COLLISION_POINTS_CAPACITY)
                {
                    break;
                }
            }

            collision_point_count = collision_points.size();

            if (collision_point_count > 0)
            {
                _project_data.collision_points_vbo->Upload(collision_points.data(), std_vector_size(collision_points));
            }
        }

        if (g_key)
        {
            glm::vec3 stretcher_position    = glm::vec3(0.0f);
            glm::mat3 stretcher_orientation = glm::mat3(1.0f);

            if (_project_data.trajectory_positions.size() && _project_data.trajectory_orientations_mat33.size())
            {
                const auto& trajectory_point      = _project_data.trajectory_positions[_project_data.trajectory_index];
                const auto& trajectoryorientation = _project_data.trajectory_orientations_mat33[_project_data.trajectory_index];

                stretcher_position    = trajectory_point.position;
                stretcher_orientation = trajectoryorientation.orientation;
            }

            glm::mat4 stretcher_pose = glm::translate(glm::mat4(1.0f), stretcher_position) * glm::mat4(stretcher_orientation);

            Viewport vp0    = ctx.viewport_for(0);
            float    rect_x = static_cast<float>(vp0.x);
            float    rect_y = static_cast<float>(height - vp0.y - vp0.h);
            float    rect_w = static_cast<float>(vp0.w);
            float    rect_h = static_cast<float>(vp0.h);

            glm::mat4 gizmo_proj = ctx.cameras[0].get_projection(rect_w, rect_h);
            glm::mat4 gizmo_view = ctx.cameras[0].get_view();

            ImGuizmo::BeginFrame();
            ImGuizmo::SetOrthographic(ctx.cameras[0].projection_type == ProjectionType::PROJECTION_TYPE_ORTHOGRAPHIC);
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
                _project_data.trajectory_positions[_project_data.trajectory_index].position             = position;
                _project_data.trajectory_orientations_mat33[_project_data.trajectory_index].orientation = rotation;

                _project_data.trajectory_positions_vbo->Upload(&position, sizeof(position), sizeof(Point) * _project_data.trajectory_index);
                // Upload the three rotation columns as tightly-packed vec3s (matching OrientationVertex layout)
                const glm::vec3 gpu_orientation[3] = {rotation[0], rotation[1], rotation[2]};
                _project_data.trajectory_orientations_vbo->Upload(gpu_orientation, sizeof(gpu_orientation), sizeof(gpu_orientation) * _project_data.trajectory_index);
            }
        }

        // One-time snap of viewport 0 camera target to the current trajectory pose (S key)
        if (s_key_clicked && !shift && _project_data.trajectory_positions.size() && !_project_data.lock_viewport0_target_to_trajectory)
        {
            snap_camera_target_to_trajectory(ctx.cameras[0], _project_data.trajectory_positions[_project_data.trajectory_index].position);
        }

        // Shift + S : toggle continuous snap on / off
        if (shift_s_clicked && _project_data.trajectory_positions.size())
        {
            _project_data.lock_viewport0_target_to_trajectory = !_project_data.lock_viewport0_target_to_trajectory;
            spdlog::info("Continuous snap to trajectory pose {}", _project_data.lock_viewport0_target_to_trajectory ? "enabled" : "disabled");
        }

        const int count = static_cast<int>(ctx.active_count);

        if (_project_data.lock_viewport0_target_to_trajectory && _project_data.trajectory_positions.size())
        {
            snap_camera_target_to_trajectory(ctx.cameras[0], _project_data.trajectory_positions[_project_data.trajectory_index].position);
        }

        for (int i = 0; i < count; ++i)
        {
            if (i >= 1 && ctx.camera_modes[i] != CameraMode::CAMERA_MODE_FREE_ORBIT)
            {
                update_locked_camera(ctx.cameras[i], ctx.camera_modes[i], ctx.view_axis_distance[i], stretcher_position, stretcher_orientation);

                // Non free look camera : planes are always derived from the current distance to the stretcher pose
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

                // Orthographic : the projection ignores the camera position, so moving the camera
                // along the view axis would not change the image. Use the view axis distance as a
                // dolly instead : the orthographic box grows / shrinks with the distance, so the
                // distance slider has the same visual effect as in the perspective case.
                if (ctx.cameras[i].projection_type == ProjectionType::PROJECTION_TYPE_ORTHOGRAPHIC)
                {
                    const float fov_half_tan         = std::tan(glm::radians(ctx.cameras[i].fov_y * 0.5f));
                    ctx.cameras[i].ortho_half_height = std::max(0.01f, ctx.view_axis_distance[i] * fov_half_tan);
                    ctx.cameras[i].ortho_zoom        = 1.0f;
                }
            }
            else if (ctx.cameras[i].up != glm::vec3(0.0f, 0.0f, 1.0f))
            {
                unlock_camera_to_free_orbit(ctx.cameras[i], ctx.view_axis_distance[i]);
            }
        }

        // VIEWPORT DIVIDER LINES
        {
            int vp_count = static_cast<int>(ctx.active_count);
            if (vp_count >= 2)
            {
                ImDrawList* dl  = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
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

        // WORLD AXES : a camera-relative orientation indicator in the bottom-left of each viewport.
        {
            ImDrawList* dl = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
            for (int i = 0; i < count; ++i)
            {
                const Viewport vp = ctx.viewport_for(i);
                if (!ctx.draw_axes_overlay[i] || vp.w <= 0 || vp.h <= 0)
                    continue;

                const float     box_w = static_cast<float>(vp.w) * ctx.axes_overlay_size[i];
                const float     box_h = static_cast<float>(vp.h) * ctx.axes_overlay_size[i];
                const ImVec2    box_min(static_cast<float>(vp.x) + 6.0f,
                                        static_cast<float>(height - vp.y) - box_h - 6.0f);
                const ImVec2    box_max(box_min.x + box_w, box_min.y + box_h);
                const ImVec2    center((box_min.x + box_max.x) * 0.5f, (box_min.y + box_max.y) * 0.5f);
                const float     axis_length = std::min(box_w, box_h) * 0.32f;
                const glm::mat3 view_rotation(ctx.cameras[i].get_view());

                struct Axis
                {
                    ImVec2      end;
                    float       depth;
                    ImU32       color;
                    const char* label;
                };
                std::array<Axis, 3> axes{};
                const glm::vec3     directions[] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
                const ImU32         colors[]     = {IM_COL32(255, 90, 90, 230), IM_COL32(90, 255, 90, 230), IM_COL32(100, 155, 255, 230)};
                const char*         labels[]     = {"X", "Y", "Z"};
                for (int axis = 0; axis < 3; ++axis)
                {
                    const glm::vec3 direction = view_rotation * directions[axis];
                    axes[axis]                = {ImVec2(center.x + direction.x * axis_length,
                                                        center.y - direction.y * axis_length),
                                                 direction.z, colors[axis], labels[axis]};
                }
                // Draw farther axes first so the ones facing the camera remain legible.
                std::sort(axes.begin(), axes.end(), [](const Axis& a, const Axis& b)
                          { return a.depth < b.depth; });

                dl->PushClipRect(box_min, box_max, true);
                dl->AddRectFilled(box_min, box_max, IM_COL32(12, 12, 12, 120), 4.0f);
                for (const Axis& axis : axes)
                {
                    dl->AddLine(center, axis.end, axis.color, 2.0f);
                    dl->AddCircleFilled(axis.end, 2.0f, axis.color);
                    dl->AddText(ImVec2(axis.end.x + 3.0f, axis.end.y - 7.0f), axis.color, axis.label);
                }
                dl->PopClipRect();
            }
        }

        // MEASUREMENT LABELS : project each midpoint through every active viewport and draw 2-D distance labels
        if (_user_settings.measurements.draw_enable)
        {
            const MeasurementState& ms = _project_data.measurements;

            if (!ms.entries.empty())
            {
                ImDrawList* dl = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());

                for (int i = 0; i < count; ++i)
                {
                    // Per-viewport toggle : measurement labels can be disabled for this viewport
                    if (!ctx.draw_measurement_labels[i])
                    {
                        continue;
                    }

                    const Camera&  cam = ctx.cameras[i];
                    const Viewport vp  = ctx.viewport_for(i);
                    if (vp.w <= 0 || vp.h <= 0)
                    {
                        continue;
                    }

                    const glm::mat4 proj = cam.get_projection(static_cast<float>(vp.w), static_cast<float>(vp.h));
                    const glm::mat4 view = cam.get_view();
                    const glm::mat4 MVP  = proj * view;

                    // Top-left corner of the viewport in ImGui (screen) coordinates
                    // OpenGL vp.y is measured from the bottom, so screen_top = height - (vp.y + vp.h)
                    const float  vp_screen_x = static_cast<float>(vp.x);
                    const float  vp_screen_y = static_cast<float>(height - (vp.y + vp.h));
                    const ImVec2 clip_min(vp_screen_x, vp_screen_y);
                    const ImVec2 clip_max(vp_screen_x + static_cast<float>(vp.w),
                                          vp_screen_y + static_cast<float>(vp.h));

                    for (size_t j = 0; j < ms.entries.size(); ++j)
                    {
                        const MeasurementEntry& e = ms.entries[j];

                        const glm::vec3 midpoint = 0.5f * (e.point_a + e.point_b);
                        const glm::vec4 clip     = MVP * glm::vec4(midpoint, 1.0f);

                        // Behind the camera → skip
                        if (clip.w <= 0.0f)
                        {
                            continue;
                        }

                        const glm::vec3 ndc = glm::vec3(clip) / clip.w;

                        // Outside the NDC cube → skip
                        if (ndc.x < -1.0f || ndc.x > 1.0f ||
                            ndc.y < -1.0f || ndc.y > 1.0f ||
                            ndc.z < -1.0f || ndc.z > 1.0f)
                        {
                            continue;
                        }

                        // NDC → ImGui screen pixel (flip Y: OpenGL Y-up, ImGui Y-down)
                        const float px = vp_screen_x + (ndc.x * 0.5f + 0.5f) * static_cast<float>(vp.w);
                        const float py = vp_screen_y + (1.0f - (ndc.y * 0.5f + 0.5f)) * static_cast<float>(vp.h);

                        char label[64];
                        std::snprintf(label, sizeof(label), "%zu: %.4f m", j + 1, e.distance_m);

                        const ImVec2 text_pos  = ImVec2(px + 4.0f, py - 8.0f);
                        const ImVec2 text_size = ImGui::CalcTextSize(label);

                        // Clip the label inside the viewport so it never bleeds into neighbouring viewports
                        dl->PushClipRect(clip_min, clip_max, true);

                        // Label : near-black background with white text, independent of the measurement line colour
                        dl->AddRectFilled(
                            ImVec2(text_pos.x - 2.0f, text_pos.y - 1.0f),
                            ImVec2(text_pos.x + text_size.x + 2.0f, text_pos.y + text_size.y + 1.0f),
                            IM_COL32(10, 10, 10, 220),
                            2.0f);

                        dl->AddText(text_pos, IM_COL32(255, 255, 255, 255), label);

                        dl->PopClipRect();
                    }
                }
            }
        }

        ImGui::Render();

        // PICKING POINT CLOUD
        {
            // Trigger once when mouse goes from released -> pressed, while Ctrl (bucket pick) or Alt (trajectory pick) is held
            bool new_click = new_left_click && (ctrl || alt);

            if ((new_click || new_right_click) && !ImGui::GetIO().WantCaptureMouse)
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

                    glm::vec3 ray_origin{};
                    glm::vec3 ray_dir{};
                    pick_cam.screen_ray(x_ndc, y_ndc, static_cast<float>(vp.w), static_cast<float>(vp.h), ray_origin, ray_dir);

                    glm::vec3 camera_pos     = pick_cam.position;
                    glm::vec3 camera_forward = glm::normalize(pick_cam.target - pick_cam.position);

                    if (alt)
                    {
                        // Pick closest trajectory point to the cast ray (within 0.25 m of the ray)
                        const float TRAJECTORY_PICK_RADIUS = 0.25f;

                        uint32_t best_index = 0;
                        float    best_dist  = TRAJECTORY_PICK_RADIUS;
                        float    best_t     = std::numeric_limits<float>::max();
                        bool     found      = false;

                        for (size_t i = 0; i < _project_data.trajectory_positions.size(); ++i)
                        {
                            const glm::vec3 to_point = _project_data.trajectory_positions[i].position - ray_origin;
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

                            if (new_right_click)
                            {
                                snap_camera_target_to_trajectory(pick_cam, _project_data.trajectory_positions[best_index].position);
                            }
                            else
                            {
                                _project_data.trajectory_index = best_index;
                            }
                        }
                        else
                        {
                            spdlog::warn("Trajectory picking missed ... (no trajectory point within {:.2f} m of ray)", TRAJECTORY_PICK_RADIUS);
                        }
                    }
                    else if (ctrl)
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
                                    if (ray_origin[i] < bmin[i] || ray_origin[i] > bmax[i])
                                    {
                                        tmin = tmax = -1.0f;
                                        break;
                                    }
                                }
                                else
                                {
                                    float invD = 1.0f / ray_dir[i];
                                    float t0   = (bmin[i] - ray_origin[i]) * invD;
                                    float t1   = (bmax[i] - ray_origin[i]) * invD;
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
                            glm::vec3 center = picked_record->aabb.min + 0.5f * (picked_record->aabb.max - picked_record->aabb.min);
                            glm::vec3 focus  = center;

                            // Ctrl + right click : pick the point in this bucket closest to the cast ray,
                            // and focus the camera target on it (instead of the bucket center)
                            if (new_right_click)
                            {
                                const float POINT_PICK_RADIUS = 0.05f;

                                bool      point_found = false;
                                float     best_d      = POINT_PICK_RADIUS;
                                float     best_t      = std::numeric_limits<float>::max();
                                glm::vec3 best_point{};

                                for (PointCloudLOD* lod = picked_record->lods; lod; lod = lod->next)
                                {
                                    for (const PointIntensity& p : lod->points)
                                    {
                                        const glm::vec3 to_point = p.position - ray_origin;
                                        const float     t        = glm::dot(to_point, ray_dir);

                                        if (t <= 0.0f)
                                        {
                                            continue;
                                        }

                                        const float d = glm::length(to_point - t * ray_dir);

                                        if (d > best_d)
                                        {
                                            continue;
                                        }

                                        if (!point_found || d < best_d - 1e-4f || (d < best_d + 1e-4f && t < best_t))
                                        {
                                            point_found = true;
                                            best_d      = d;
                                            best_t      = t;
                                            best_point  = p.position;
                                        }
                                    }
                                }

                                if (point_found)
                                {
                                    focus = best_point;
                                    spdlog::info("Point pick in viewport {} : bucket [{} {} {}], point ({:.3f}, {:.3f}, {:.3f}) (distance to ray {:.3f} m)", pick_idx, picked_id.x, picked_id.y, picked_id.z, focus.x, focus.y, focus.z, best_d);
                                }
                                else
                                {
                                    spdlog::warn("Point picking missed ... (no point within {:.2f} m of ray in bucket [{} {} {}])", POINT_PICK_RADIUS, picked_id.x, picked_id.y, picked_id.z);
                                }
                            }
                            else
                            {
                                spdlog::info("Picking hit in viewport {} : ID = [{} {} {}]", pick_idx, picked_id.x, picked_id.y, picked_id.z);
                            }

                            glm::vec3 offset  = pick_cam.position - pick_cam.target;
                            pick_cam.target   = focus;
                            pick_cam.position = focus + offset;
                        }
                        else
                        {
                            spdlog::warn("Picking missed ...");
                        }
                    }
                }
            }
        }

        // MEASUREMENT PICKING (Shift + LMB)
        {
            if (new_left_click && shift && !ImGui::GetIO().WantCaptureMouse)
            {
                double mouse_x, mouse_y;
                glfwGetCursorPos(window, &mouse_x, &mouse_y);

                int pick_idx = ctx.camera_index_at(mouse_x, mouse_y);
                if (pick_idx >= 0 && !_user_settings.measurements.draw_enable)
                {
                    spdlog::warn("Measurement picking blocked: measurement display is disabled");
                }
                else if (pick_idx >= 0)
                {
                    Camera&  pick_cam = ctx.cameras[pick_idx];
                    Viewport vp       = ctx.viewport_for(pick_idx);

                    double local_x    = mouse_x - vp.x;
                    double local_gl_y = (static_cast<double>(height) - mouse_y) - vp.y;

                    float x_ndc = (2.0f * static_cast<float>(local_x) / static_cast<float>(vp.w)) - 1.0f;
                    float y_ndc = (2.0f * static_cast<float>(local_gl_y) / static_cast<float>(vp.h)) - 1.0f;

                    glm::vec3 ray_origin{};
                    glm::vec3 ray_dir{};
                    pick_cam.screen_ray(x_ndc, y_ndc, static_cast<float>(vp.w), static_cast<float>(vp.h), ray_origin, ray_dir);

                    glm::vec3 camera_pos     = pick_cam.position;
                    glm::vec3 camera_forward = glm::normalize(pick_cam.target - pick_cam.position);

                    // Find the closest point cloud point to the ray, respecting near/far planes
                    const float MEAS_PICK_RADIUS = 0.05f;

                    bool      point_found = false;
                    float     best_d      = MEAS_PICK_RADIUS;
                    float     best_t      = std::numeric_limits<float>::max();
                    glm::vec3 best_point{};

                    for (auto& [ID, bucket] : _project_data.buckets)
                    {
                        for (PointCloudLOD* lod = bucket.lods; lod; lod = lod->next)
                        {
                            for (const PointIntensity& p : lod->points)
                            {
                                const glm::vec3 to_point = p.position - ray_origin;

                                // t = projection onto the ray
                                const float t = glm::dot(to_point, ray_dir);

                                // Respect camera near/far: the point must be visible
                                // (its depth along camera_forward must lie within [near_plane, far_plane])
                                const float depth = glm::dot(p.position - camera_pos, camera_forward);
                                if (depth < pick_cam.near_plane || depth > pick_cam.far_plane)
                                {
                                    continue;
                                }

                                if (t <= 0.0f)
                                {
                                    continue;
                                }

                                const float d = glm::length(to_point - t * ray_dir);

                                if (d > best_d)
                                {
                                    continue;
                                }

                                if (!point_found || d < best_d - 1e-4f || (d < best_d + 1e-4f && t < best_t))
                                {
                                    point_found = true;
                                    best_d      = d;
                                    best_t      = t;
                                    best_point  = p.position;
                                }
                            }
                        }
                    }

                    if (point_found)
                    {
                        MeasurementState& ms = _project_data.measurements;

                        if (!ms.pending_point.has_value())
                        {
                            // First pick : store the start point
                            ms.pending_point = best_point;
                            spdlog::info("Measurement pick A in viewport {} : ({:.3f}, {:.3f}, {:.3f})", pick_idx, best_point.x, best_point.y, best_point.z);
                        }
                        else
                        {
                            // Second pick : complete the measurement
                            MeasurementEntry entry;
                            entry.point_a    = ms.pending_point.value();
                            entry.point_b    = best_point;
                            entry.distance_m = glm::length(entry.point_b - entry.point_a);

                            if (ms.entries.size() < ProjectData::MEASUREMENT_LINE_CAPACITY)
                            {
                                ms.entries.push_back(entry);
                            }
                            else
                            {
                                spdlog::warn("Measurement capacity reached ({} entries) – clear some before adding more", ProjectData::MEASUREMENT_LINE_CAPACITY);
                            }

                            ms.pending_point.reset();
                            spdlog::info("Measurement pick B in viewport {} : ({:.3f}, {:.3f}, {:.3f}) | distance = {:.4f} m", pick_idx, best_point.x, best_point.y, best_point.z, entry.distance_m);
                        }
                    }
                    else
                    {
                        spdlog::warn("Measurement picking missed (no point within {:.3f} m of ray in viewport {})", MEAS_PICK_RADIUS, pick_idx);
                    }
                }
            }

            // Upload measurement line vertices to GPU each frame
            if (_project_data.measurement_line_vbo)
            {
                const MeasurementState& ms = _project_data.measurements;

                static std::vector<ColoredVertex> meas_verts;
                meas_verts.clear();
                for (const MeasurementEntry& e : ms.entries)
                {
                    meas_verts.push_back({e.point_a, e.color});
                    meas_verts.push_back({e.point_b, e.color});
                }
                if (!meas_verts.empty())
                {
                    _project_data.measurement_line_vbo->Upload(meas_verts.data(), meas_verts.size() * sizeof(ColoredVertex));
                }
            }
        }

        auto draw_scene = [&](const uint32_t viewport_index, const Viewport& vp, Camera& cam)
        {
            const int pixel_x = static_cast<int>(static_cast<int64_t>(vp.x) * framebuffer_width / width);
            const int pixel_y = static_cast<int>(static_cast<int64_t>(vp.y) * framebuffer_height / height);
            const int pixel_w = static_cast<int>(static_cast<int64_t>(vp.x + vp.w) * framebuffer_width / width) - pixel_x;
            const int pixel_h = static_cast<int>(static_cast<int64_t>(vp.y + vp.h) * framebuffer_height / height) - pixel_y;
            glViewport(pixel_x, pixel_y, pixel_w, pixel_h);

            glm::mat4 projection = cam.get_projection(static_cast<float>(vp.w), static_cast<float>(vp.h));
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
                trajectory_program->Bind();
                trajectory_program->PushUniform16F32("u_MVP", MVP);
                trajectory_program->PushUniform3F32("u_Color", _user_settings.trajectory.color);
                _project_data.trajectory_positions_vao->Bind();

                if (_user_settings.trajectory.display_mode == TrajectoryDisplayMode::TRAJECTORY_DISPLAY_MODE_POINTS)
                {
                    glPointSize(_user_settings.trajectory.point_size);
                    _project_data.trajectory_positions_vao->DrawArray(GL_POINTS, _project_data.trajectory_positions.size());
                    glPointSize(1.0f);
                }
                else
                {
                    glLineWidth(_user_settings.trajectory.width);
                    _project_data.trajectory_positions_vao->DrawArray(GL_LINE_STRIP, _project_data.trajectory_positions.size());
                    glLineWidth(1.0f);
                }
                if (_user_settings.trajectory.draw_orientations && _project_data.trajectory_axes_vao)
                {
                    trajectory_axes_program->Bind();
                    trajectory_axes_program->PushUniform16F32("u_MVP", MVP);
                    trajectory_axes_program->PushUniform1F32("u_AxisLength", _user_settings.trajectory.orientation_axis_length);
                    _project_data.trajectory_axes_vao->Bind();
                    _project_data.trajectory_axes_vao->DrawArray(GL_POINTS, _project_data.trajectory_positions.size());
                }
            }

            // MEASUREMENT LINES : draw completed entries as 3-D lines in world space
            if (_user_settings.measurements.draw_enable)
            {
                const MeasurementState& ms          = _project_data.measurements;
                const size_t            entry_count = ms.entries.size();

                if (entry_count > 0 && _project_data.measurement_line_vao)
                {
                    // Per-measurement colours come from the vertex buffer
                    colored_line_program->Bind();
                    colored_line_program->PushUniform16F32("u_MVP", MVP);
                    _project_data.measurement_line_vao->Bind();

                    for (size_t i = 0; i < entry_count; ++i)
                    {
                        glLineWidth(ms.entries[i].line_width);
                        // Draw 2 vertices for each line segment starting at index i * 2
                        _project_data.measurement_line_vao->DrawArray(GL_LINES, static_cast<uint32_t>(i * 2), 2);
                    }
                    glLineWidth(1.0f);
                }

                // MEASUREMENT PENDING POINT : draw a small yellow cross at the first picked point
                if (ms.pending_point.has_value() && _user_settings.target.draw_enable)
                {
                    camera_target_program->Bind();
                    camera_target_program->PushUniform16F32("u_MVP", MVP);
                    camera_target_program->PushUniform3F32("u_Translation", ms.pending_point.value());
                    camera_target_program->PushUniform1F32("u_Scale", _user_settings.target.scale * 0.5f);
                    camera_target_program->PushUniform3F32("u_Color", glm::vec3(1.0f, 1.0f, 0.0f));
                    target_vao->Bind();
                    target_vao->DrawArray(GL_LINES, 6);
                }
            }

            //  STRETCHER
            if (_user_settings.stretcher.draw_enable && (_project_data.stretcher_vertices.size() && _project_data.stretcher_indices.size()))
            {
                stretcher_program->Bind();
                stretcher_program->PushUniform16F32("u_MVP", MVP);
                stretcher_program->PushUniform16F32("u_Pose", stretcher_pose);

                _project_data.stretcher_vao->Bind();
                _project_data.stretcher_vao->DrawElements(GL_TRIANGLES, _project_data.stretcher_indices.size());
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

            // COLLISION POINTS : first-LOD points inside the stretcher OBB, drawn with configurable color and point size
            if (_user_settings.point_cloud.draw_enable_pc && _project_data.buckets.size() && collision_point_count > 0)
            {
                glPointSize(_user_settings.collision.points_size);

                // Trajectory program : u_MVP + u_Color with position-only layout, matches collision points VAO
                trajectory_program->Bind();
                trajectory_program->PushUniform16F32("u_MVP", MVP);
                trajectory_program->PushUniform3F32("u_Color", _user_settings.collision.points_color);

                _project_data.collision_points_vao->Bind();
                _project_data.collision_points_vao->DrawArray(GL_POINTS, static_cast<uint32_t>(collision_point_count));

                glPointSize(1.0f);
            }

            // POINT_CLOUD
            if (_user_settings.point_cloud.draw_enable_pc && _project_data.buckets.size() && draw_any_cave_lod)
            {
                glm::vec3 camera_pos = glm::vec3(glm::inverse(view)[3]);

                glPointSize(_user_settings.point_cloud.point_size);

                const bool use_color_map    = _user_settings.point_cloud.display_mode != PointCloudDisplayMode::POINT_CLOUD_DISPLAY_MODE_INTENSITY;
                const bool use_position_map = _user_settings.point_cloud.display_mode == PointCloudDisplayMode::POINT_CLOUD_DISPLAY_MODE_COLOR_MAP_POSITION ||
                                              _user_settings.point_cloud.display_mode == PointCloudDisplayMode::POINT_CLOUD_DISPLAY_MODE_COLOR_MAP_POSITION_TIMES_INTENSITY;

                Program* active_point_cloud_program = use_color_map ? point_cloud_color_map_program : point_cloud_program;
                active_point_cloud_program->Bind();
                active_point_cloud_program->PushUniform16F32("u_MVP", MVP);

                if (use_color_map)
                {
                    active_point_cloud_program->PushUniformS32("u_ColorMapSelect", static_cast<int32_t>(_user_settings.point_cloud.colormap));
                    active_point_cloud_program->PushUniform1F32("u_IntensityMin", _project_data.intensity_min);
                    active_point_cloud_program->PushUniform1F32("u_IntensityInvRange", (_project_data.intensity_max > _project_data.intensity_min) ? 1.0f / (_project_data.intensity_max - _project_data.intensity_min) : 1.0f);
                    active_point_cloud_program->PushUniform1F32("u_UsePosition", use_position_map ? 1.0f : 0.0f);
                    active_point_cloud_program->PushUniform3F32("u_PositionMin", _project_data.cave_aabb.min);
                    const glm::vec3 position_extent = glm::max(_project_data.cave_aabb.max - _project_data.cave_aabb.min, glm::vec3(1e-6f));
                    active_point_cloud_program->PushUniform3F32("u_PositionInvRange", 1.0f / position_extent);
                    active_point_cloud_program->PushUniform1F32("u_MultiplyIntensity", _user_settings.point_cloud.display_mode == PointCloudDisplayMode::POINT_CLOUD_DISPLAY_MODE_COLOR_MAP_TIMES_INTENSITY ||
                                                                                               _user_settings.point_cloud.display_mode == PointCloudDisplayMode::POINT_CLOUD_DISPLAY_MODE_COLOR_MAP_POSITION_TIMES_INTENSITY
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

                    const size_t   viewport_i = std::min<size_t>(viewport_index, MultiViewContext::MAX_CAMERAS - 1);
                    size_t         lod_index  = _project_data.multi_view.use_fixed_lod[viewport_i] ? static_cast<size_t>(_project_data.multi_view.fixed_lod_index[viewport_i]) : lod_from_distance(distance, 70.0f, lod_count);
                    PointCloudLOD* lod        = get_lod_at_index(&bucket, lod_index);

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

        for (int i = 0; i < count; ++i)
        {
            draw_scene(i, ctx.viewport_for(i), ctx.cameras[i]);
        }

        // When viewports use different projection types the camera pass must be repeated so the
        // projected pixel sizes used for LOD selection match each viewport's projection.
        if (any_projection_differs(ctx, count))
        {
            for (int i = count; i < MultiViewContext::MAX_CAMERAS; ++i)
            {
                draw_scene(i, ctx.viewport_for(i), ctx.cameras[i]);
            }
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
