#pragma once

#include <Core/Enums.h>

#include <array>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

struct Camera
{
    glm::vec3 position = glm::vec3(10.0f, 10.0f, 10.0f);
    glm::vec3 target   = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 up       = glm::vec3(0.0f, 0.0f, 1.0f);

    bool       rotating    = false;
    bool       panning     = false;
    glm::dvec2 last_cursor = {0.0, 0.0};

    // Projection type of this camera
    ProjectionType projection_type = ProjectionType::PROJECTION_TYPE_PERSPECTIVE;

    // Perspective : vertical field of view in degrees
    float fov_y      = 45.0f;
    float viewport_w = 800;
    float viewport_h = 600;

    // Perspective : near / far distances from the camera (near > 0)
    // Orthographic : near / far can be negative, the box spans [near, far] along the view axis
    float near_plane = 0.1f;
    float far_plane  = 1000.0f;

    // Orthographic only : view-space half extents of the projection box
    // (symmetric around the view axis). The vertical extent is derived from
    // ortho_half_height and the aspect ratio, matching the perspective framing.
    float ortho_half_height = 5.0f;

    // Orthographic zoom : multiplies the vertical world extent mapped to the viewport.
    // > 1 zooms in ("magnifies"), < 1 zooms out. Used for scroll wheel / pan speed.
    float ortho_zoom = 1.0f;

    glm::mat4 get_view() const;

    // Projection matrix for the current projection_type and the given viewport aspect ratio
    glm::mat4 get_projection(float viewport_width, float viewport_height) const;

    // Orthographic only : view-space half extents (width, height) of the projection box
    std::array<float, 2> ortho_extents(float viewport_width, float viewport_height) const;

    // World units per pixel at a given depth along the view axis (handles both projection types)
    float world_units_per_pixel(float depth) const;

    // Picking ray for a normalized device coordinate (ndc_x, ndc_y in [-1, 1]).
    // Perspective : the ray starts at the camera position and diverges.
    // Orthographic : the ray direction is the camera forward for every pixel, only the
    // origin shifts across the orthographic box (a parallel ray), matching what is rendered.
    // out_origin / out_direction (direction normalized) are in world space.
    void screen_ray(float ndc_x, float ndc_y, float viewport_width, float viewport_height,
                    glm::vec3& out_origin, glm::vec3& out_direction) const;

    void rotate(double dx, double dy);
    void pan(double dx, double dy);
    void zoom(double scroll);
};

struct Viewport
{
    int x{}, y{};
    int w{}, h{};
};

struct MultiViewContext
{
    static constexpr int MAX_CAMERAS = 4;

    Camera cameras[MAX_CAMERAS]{};

    CameraMode camera_modes[MAX_CAMERAS]{
        CameraMode::CAMERA_MODE_FREE_ORBIT,
        CameraMode::CAMERA_MODE_LOCAL_X,
        CameraMode::CAMERA_MODE_LOCAL_Y,
        CameraMode::CAMERA_MODE_LOCAL_Z};

    // Locked-axis cameras only : axis length = distance of the camera from the stretcher pose along the view axis.
    // The near / far planes are the regular Camera::near_plane / Camera::far_plane depths measured from the camera.

    float view_axis_distance[MAX_CAMERAS]{
        5.0f,
        5.0f,
        5.0f,
        5.0f};

    // Per-viewport plane mode (locked-axis cameras only)
    // true  : symmetric  - near / far planes are derived every frame as axis length -+ symmetric_plane_offset
    // false : asymmetric - near / far planes are controlled independently of the axis length

    bool symmetric_planes[MAX_CAMERAS]{
        true,
        true,
        true,
        true};

    float symmetric_plane_offset[MAX_CAMERAS]{
        1.25f,
        1.25f,
        1.25f,
        1.25f};

    ViewportCount active_count{ViewportCount::VIEWPORT_COUNT_FOUR};

    // Layout used when active_count == 4 : 2x2 grid or 1 large viewport + 3 stacked on the right
    ViewportLayout layout{ViewportLayout::VIEWPORT_LAYOUT_ONE_BIG};

    // Large-viewport layout : fraction of the window width taken by the large viewport (0.5 .. 0.75, default 2/3)
    float large_viewport_fraction{2.0f / 3.0f};

    // Level of detail : fixed LOD index vs automatic LOD from distance (per viewport / camera)

    bool use_fixed_lod[MAX_CAMERAS]{
        true,
        true,
        true,
        true};

    int32_t fixed_lod_index[MAX_CAMERAS]{
        0,
        0,
        0,
        0};

    // World axes orientation overlay in the bottom-left of each viewport
    // size is a fraction of the viewport width and height

    bool draw_axes_overlay[MAX_CAMERAS]{
        true,
        true,
        true,
        true};

    float axes_overlay_size[MAX_CAMERAS]{
        0.1f,
        0.1f,
        0.1f,
        0.1f};

    // Per-viewport drawing of measurement labels (measured value tags)
    // Default : on for viewport 1, off for viewports 2, 3 and 4

    bool draw_measurement_labels[MAX_CAMERAS]{
        true,
        false,
        false,
        false};

    int window_width{800};
    int window_height{600};

    Viewport viewport_for(int i) const;
    int      camera_index_at(double xpos, double ypos) const;
};

glm::vec3 axis_camera_offset(CameraMode mode, float distance, const glm::mat3& orientation);

void update_locked_camera(Camera& camera, CameraMode mode, float distance, const glm::vec3& target, const glm::mat3& orientation);
void unlock_camera_to_free_orbit(Camera& camera, float fallback_distance = 5.0f);

void snap_camera_target_to_trajectory(Camera& cam, const glm::vec3& pose_pos);

struct GLFWwindow;

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void size_callback(GLFWwindow* window, int32_t width, int32_t height);

void cursor_position_callback(GLFWwindow* window, MultiViewContext& context, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, MultiViewContext& context, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, MultiViewContext& context, double xoffset, double yoffset);
void size_callback(GLFWwindow* window, MultiViewContext& context, int32_t width, int32_t height);
