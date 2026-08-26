#include <cave-traversal-tool/Camera.h>

// clang-format off
#include <GLFW/glfw3.h>
#include <imgui.h>
// clang-format on

glm::mat4 Camera::get_view() const
{
    return glm::lookAt(position, target, up);
}

void Camera::rotate(double dx, double dy)
{
    glm::vec3 offset   = position - target;
    glm::vec3 forward  = glm::normalize(-offset);
    glm::vec3 right    = glm::normalize(glm::cross(forward, up));
    glm::vec3 local_up = glm::normalize(glm::cross(right, forward));

    float angle_x = static_cast<float>(-dx * 0.005);
    float angle_y = static_cast<float>(-dy * 0.005);

    glm::mat4 rot_h = glm::rotate(glm::mat4(1.0f), angle_x, local_up);
    glm::mat4 rot_v = glm::rotate(glm::mat4(1.0f), angle_y, right);

    offset = glm::vec3(rot_v * rot_h * glm::vec4(offset, 1.0f));

    glm::vec3 new_forward = glm::normalize(-offset);
    float     pitch       = glm::degrees(glm::asin(new_forward.y));

    if (pitch < 89.0f && pitch > -89.0f)
    {
        position = target + offset;
    }
}

void Camera::pan(double dx, double dy)
{
    glm::vec3 forward  = glm::normalize(target - position);
    glm::vec3 right    = glm::normalize(glm::cross(forward, up));
    glm::vec3 local_up = glm::normalize(glm::cross(right, forward));

    float scale = 2.0f * tan(glm::radians(fov_y * 0.5f)) / viewport_h;

    glm::vec3 move = static_cast<float>(-dx) * scale * right + static_cast<float>(dy) * scale * local_up;

    position += move;
    target += move;
}

void Camera::zoom(double scroll)
{
    glm::vec3 forward     = glm::normalize(target - position);
    float     zoom_amount = static_cast<float>(scroll) * 0.5f;

    float distance     = glm::length(target - position);
    float min_distance = 0.1f;

    float new_distance = distance - zoom_amount;

    if (new_distance < min_distance)
    {
        zoom_amount = distance - min_distance;
    }

    position += forward * zoom_amount;
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    Camera* camera = (Camera*)glfwGetWindowUserPointer(window);

    if (!camera)
    {
        return;
    }

    glm::dvec2 delta = glm::dvec2(xpos, ypos) - camera->last_cursor;

    if (camera->rotating)
    {
        camera->rotate(delta.x, delta.y);
    }

    if (camera->panning)
    {
        float panning_multiplier = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) ? 50.0f : 10.0f;

        // for HUGE dataset
        delta *= panning_multiplier;

        camera->pan(delta.x, delta.y);
    }

    camera->last_cursor = {xpos, ypos};
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    Camera* camera = (Camera*)glfwGetWindowUserPointer(window);

    if (!camera)
    {
        return;
    }

    if (ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        camera->rotating = (action == GLFW_PRESS);
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        camera->panning = (action == GLFW_PRESS);
    }

    glfwGetCursorPos(window, &camera->last_cursor.x, &camera->last_cursor.y);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    Camera* camera = (Camera*)glfwGetWindowUserPointer(window);

    if (!camera)
    {
        return;
    }

    if (ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    float scroll_multiplier = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) ? 10.0f : 1.0f;

    camera->zoom(yoffset * scroll_multiplier);
}

void size_callback(GLFWwindow* window, int32_t width, int32_t height)
{
    Camera* camera = (Camera*)glfwGetWindowUserPointer(window);

    if (!camera)
    {
        return;
    }

    camera->viewport_w = width;
    camera->viewport_h = height;
}