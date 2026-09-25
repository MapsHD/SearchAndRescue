#include <cave-traversal-tool/Camera.h>

#include <algorithm>

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
    glm::vec3 cross_fu = glm::cross(forward, up);
    if (glm::length(cross_fu) < 1e-4f)
    {
        cross_fu = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    glm::vec3 right    = glm::normalize(cross_fu);
    glm::vec3 local_up = glm::normalize(glm::cross(right, forward));

    float angle_x = static_cast<float>(-dx * 0.005);
    float angle_y = static_cast<float>(-dy * 0.005);

    glm::mat4 rot_h = glm::rotate(glm::mat4(1.0f), angle_x, local_up);
    glm::mat4 rot_v = glm::rotate(glm::mat4(1.0f), angle_y, right);

    offset = glm::vec3(rot_v * rot_h * glm::vec4(offset, 1.0f));

    glm::vec3 new_forward = glm::normalize(-offset);
    // float     pitch       = glm::degrees(glm::asin(new_forward.y));
    float pitch = glm::degrees(glm::asin(glm::clamp(glm::dot(new_forward, up), -1.0f, 1.0f)));

    if (pitch < 89.0f && pitch > -89.0f)
    {
        position = target + offset;
    }
}

void Camera::pan(double dx, double dy)
{
    glm::vec3 forward  = glm::normalize(target - position);
    glm::vec3 cross_fu = glm::cross(forward, up);
    if (glm::length(cross_fu) < 1e-4f)
    {
        cross_fu = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    glm::vec3 right    = glm::normalize(cross_fu);
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

Viewport MultiViewContext::viewport_for(int i) const
{
    int hw = window_width / 2;
    int hh = window_height / 2;

    switch (static_cast<int>(active_count))
    {
    case 1:
        return {0, 0, window_width, window_height};

    case 2:
        // Slot 0 = left, slot 1 = right
        return {i * hw, 0, hw, window_height};

    case 4:
        // Slots: 0=top-left, 1=top-right, 2=bottom-left, 3=bottom-right
        // OpenGL origin is bottom-left, so top row lives at y = hh
        {
            int col  = i % 2;
            int row  = i / 2; // 0 = top row, 1 = bottom row
            int vp_y = (row == 0) ? hh : 0;
            return {col * hw, vp_y, hw, hh};
        }

    default:
        return {0, 0, window_width, window_height};
    }
}

int MultiViewContext::camera_index_at(double xpos, double ypos) const
{
    // Convert GLFW top-left Y to OpenGL bottom-left Y
    double gl_y = static_cast<double>(window_height) - ypos;

    int count = static_cast<int>(active_count);
    for (int i = 0; i < count; ++i)
    {
        Viewport vp = viewport_for(i);
        if (xpos >= vp.x && xpos < vp.x + vp.w &&
            gl_y >= vp.y && gl_y < vp.y + vp.h)
        {
            return i;
        }
    }
    return -1;
}

glm::vec3 axis_camera_offset(CameraMode mode, float distance, const glm::mat3& orientation)
{
    const glm::vec3 world_axis = [mode]()
    {
        switch (mode)
        {
        case CameraMode::AXIS_X:
        case CameraMode::LOCAL_X:
            return glm::vec3(1.0f, 0.0f, 0.0f);
        case CameraMode::AXIS_NX:
        case CameraMode::LOCAL_NX:
            return glm::vec3(-1.0f, 0.0f, 0.0f);
        case CameraMode::AXIS_Y:
        case CameraMode::LOCAL_Y:
            return glm::vec3(0.0f, 1.0f, 0.0f);
        case CameraMode::AXIS_NY:
        case CameraMode::LOCAL_NY:
            return glm::vec3(0.0f, -1.0f, 0.0f);
        case CameraMode::AXIS_Z:
        case CameraMode::LOCAL_Z:
            return glm::vec3(0.0f, 0.0f, 1.0f);
        case CameraMode::AXIS_NZ:
        case CameraMode::LOCAL_NZ:
            return glm::vec3(0.0f, 0.0f, -1.0f);
        default:
            return glm::vec3(0.0f, 0.0f, 0.0f);
        }
    }();

    const bool is_local = (mode >= CameraMode::LOCAL_X);

    return (is_local ? orientation * world_axis : world_axis) * distance;
}

void update_locked_camera(Camera& camera, CameraMode mode, float distance, const glm::vec3& target, const glm::mat3& orientation)
{
    camera.target = target;

    if (mode == CameraMode::AXIS_Z || mode == CameraMode::AXIS_NZ)
    {
        camera.up = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    else if (mode == CameraMode::LOCAL_Z || mode == CameraMode::LOCAL_NZ)
    {
        camera.up = orientation * glm::vec3(0.0f, 1.0f, 0.0f);
    }
    else if (mode == CameraMode::LOCAL_X || mode == CameraMode::LOCAL_NX || mode == CameraMode::LOCAL_Y || mode == CameraMode::LOCAL_NY)
    {
        camera.up = orientation * glm::vec3(0.0f, 0.0f, 1.0f);
    }
    else
    {
        camera.up = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    camera.position = target + axis_camera_offset(mode, distance, orientation);
}

void unlock_camera_to_free_orbit(Camera& camera, float fallback_distance)
{
    camera.up = glm::vec3(0.0f, 0.0f, 1.0f);

    glm::vec3 offset = camera.position - camera.target;
    float     dist   = glm::length(offset);
    if (dist < 0.001f)
    {
        dist   = fallback_distance > 0.1f ? fallback_distance : 5.0f;
        offset = glm::vec3(dist, 0.0f, 0.0f);
    }

    glm::vec3 dir   = offset / dist;
    float     dot_z = glm::dot(dir, glm::vec3(0.0f, 0.0f, 1.0f));

    // If looking nearly straight along Z (+Z or -Z), tilt slightly off the pole (e.g. ~75 deg pitch)
    // so pitch is within safe bounds (-89 deg, 89 deg) and cross(forward, up) is non-zero
    if (dot_z > 0.98f)
    {
        dir = glm::normalize(glm::vec3(0.0f, -std::sin(glm::radians(15.0f)), std::cos(glm::radians(15.0f))));
    }
    else if (dot_z < -0.98f)
    {
        dir = glm::normalize(glm::vec3(0.0f, -std::sin(glm::radians(15.0f)), -std::cos(glm::radians(15.0f))));
    }

    camera.position = camera.target + dir * dist;
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    auto* ctx = static_cast<MultiViewContext*>(glfwGetWindowUserPointer(window));
    if (!ctx)
    {
        return;
    }

    int active_idx = -1;
    for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
    {
        if (ctx->cameras[i].rotating || ctx->cameras[i].panning)
        {
            active_idx = i;
            break;
        }
    }

    if (active_idx >= 0 && ctx->camera_modes[active_idx] == CameraMode::FREE_ORBIT)
    {
        Camera&    camera = ctx->cameras[active_idx];
        glm::dvec2 delta  = glm::dvec2(xpos, ypos) - camera.last_cursor;

        if (camera.rotating)
        {
            camera.rotate(delta.x, delta.y);
        }

        if (camera.panning)
        {
            float panning_multiplier = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) ? 50.0f : 10.0f;
            delta *= panning_multiplier;
            camera.pan(delta.x, delta.y);
        }
    }

    for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
    {
        ctx->cameras[i].last_cursor = {xpos, ypos};
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    auto* ctx = static_cast<MultiViewContext*>(glfwGetWindowUserPointer(window));
    if (!ctx)
    {
        return;
    }

    if (action == GLFW_RELEASE)
    {
        for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
        {
            if (button == GLFW_MOUSE_BUTTON_LEFT)
            {
                ctx->cameras[i].rotating = false;
            }
            if (button == GLFW_MOUSE_BUTTON_RIGHT)
            {
                ctx->cameras[i].panning = false;
            }
        }
        return;
    }

    if (ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    int idx = ctx->camera_index_at(xpos, ypos);

    if (idx < 0 || ctx->camera_modes[idx] != CameraMode::FREE_ORBIT)
    {
        return;
    }

    Camera& camera = ctx->cameras[idx];

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        camera.rotating = true;
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        camera.panning = true;
    }

    for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
    {
        ctx->cameras[i].last_cursor = {xpos, ypos};
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto* ctx = static_cast<MultiViewContext*>(glfwGetWindowUserPointer(window));
    if (!ctx || ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    int idx = ctx->camera_index_at(xpos, ypos);

    if (idx < 0 || ctx->camera_modes[idx] != CameraMode::FREE_ORBIT)
    {
        return;
    }

    float scroll_multiplier = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) ? 10.0f : 1.0f;

    // For locked-axis cameras, scrolling adjusts the view axis length
    // (trajectory position -> camera distance) instead of free zoom.
    if (ctx->camera_modes[idx] != CameraMode::FREE_ORBIT)
    {
        ctx->view_axis_distance[idx] = std::max(0.1f, ctx->view_axis_distance[idx] - static_cast<float>(yoffset) * 0.5f * scroll_multiplier);
        return;
    }

    ctx->cameras[idx].zoom(yoffset * scroll_multiplier);
}

void size_callback(GLFWwindow* window, int32_t width, int32_t height)
{
    auto* ctx = static_cast<MultiViewContext*>(glfwGetWindowUserPointer(window));
    if (!ctx)
    {
        return;
    }

    ctx->window_width  = width;
    ctx->window_height = height;

    // Update each camera's viewport dimensions used by pan()
    for (int i = 0; i < MultiViewContext::MAX_CAMERAS; ++i)
    {
        Viewport vp                = ctx->viewport_for(i);
        ctx->cameras[i].viewport_w = static_cast<float>(vp.w);
        ctx->cameras[i].viewport_h = static_cast<float>(vp.h);
    }
}
