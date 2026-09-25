#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

struct Camera
{
    glm::vec3 position = glm::vec3(10.0f, 10.0f, 10.0f);
    glm::vec3 target   = glm::vec3(0.0f, 0.0f, 0.0f);
    // glm::vec3 up       = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f);

    bool       rotating    = false;
    bool       panning     = false;
    glm::dvec2 last_cursor = {0.0, 0.0};

    float fov_y      = 45.0f;
    float viewport_w = 800;
    float viewport_h = 600;
    float near_plane = 0.1f;
    float far_plane  = 1000.0f;

    glm::mat4 get_view() const;
    void      rotate(double dx, double dy);
    void      pan(double dx, double dy);
    void      zoom(double scroll);
};

enum class CameraMode : int
{
    FREE_ORBIT = 0,
    AXIS_X,
    AXIS_NX,
    AXIS_Y,
    AXIS_NY,
    AXIS_Z,
    AXIS_NZ,
    LOCAL_X,
    LOCAL_NX,
    LOCAL_Y,
    LOCAL_NY,
    LOCAL_Z,
    LOCAL_NZ,
};

enum class ViewportCount : int
{
    ONE  = 1,
    TWO  = 2,
    FOUR = 4
};

struct Viewport
{
    int x{}, y{};
    int w{}, h{};
};

struct MultiViewContext
{
    static constexpr int MAX_CAMERAS = 4;

    Camera     cameras[MAX_CAMERAS]{};
    CameraMode camera_modes[MAX_CAMERAS]{
        CameraMode::FREE_ORBIT,
        CameraMode::FREE_ORBIT,
        CameraMode::FREE_ORBIT,
        CameraMode::FREE_ORBIT,
    };

    float view_axis_distance[MAX_CAMERAS]{5.0f, 5.0f, 5.0f, 5.0f};

    ViewportCount active_count{ViewportCount::ONE};
    int           window_width{800};
    int           window_height{600};

    Viewport viewport_for(int i) const;
    int      camera_index_at(double xpos, double ypos) const;
};

glm::vec3 axis_camera_offset(CameraMode mode, float distance, const glm::mat3& orientation);

void update_locked_camera(Camera& camera, CameraMode mode, float distance, const glm::vec3& target, const glm::mat3& orientation);
void unlock_camera_to_free_orbit(Camera& camera, float fallback_distance = 5.0f);

struct GLFWwindow;

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void size_callback(GLFWwindow* window, int32_t width, int32_t height);
