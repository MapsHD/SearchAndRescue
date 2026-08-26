#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

struct Camera
{
    glm::vec3 position = glm::vec3(10.0f, 10.0f, 10.0f);
    glm::vec3 target   = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 up       = glm::vec3(0.0f, 1.0f, 0.0f);

    bool       rotating    = false;
    bool       panning     = false;
    glm::dvec2 last_cursor = {0.0, 0.0};

    float fov_y      = 45.0f;
    float viewport_w = 800;
    float viewport_h = 600;

    glm::mat4 get_view() const;
    void      rotate(double dx, double dy);
    void      pan(double dx, double dy);
    void      zoom(double scroll);
};

struct GLFWwindow;

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void size_callback(GLFWwindow* window, int32_t width, int32_t height);