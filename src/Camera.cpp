#include <cave-traversal-tool/Camera.h>

#include <algorithm>
#include <cmath>

// clang-format off
#include <GLFW/glfw3.h>
#include <imgui.h>
// clang-format on

glm::mat4 Camera::get_view() const
{
    return glm::lookAt(position, target, up);
}

std::array<float, 2> Camera::ortho_extents(float viewport_width, float viewport_height) const
{
    const float aspect = viewport_height > 0.0f ? viewport_width / viewport_height : 1.0f;
    const float half_h = std::max(1e-4f, ortho_half_height / std::max(1e-4f, ortho_zoom));
    return {half_h * aspect, half_h};
}

glm::mat4 Camera::get_projection(float viewport_width, float viewport_height) const
{
    if (projection_type == ProjectionType::ORTHOGRAPHIC)
    {
        const auto [half_w, half_h] = ortho_extents(viewport_width, viewport_height);
        return glm::ortho(-half_w, half_w, -half_h, half_h, near_plane, far_plane);
    }

    return glm::perspectiveFov(glm::radians(fov_y),
                               std::max(1.0f, viewport_width), std::max(1.0f, viewport_height),
                               near_plane, far_plane);
}

float Camera::world_units_per_pixel(float depth) const
{
    if (projection_type == ProjectionType::ORTHOGRAPHIC)
    {
        const float half_h = std::max(1e-4f, ortho_half_height / std::max(1e-4f, ortho_zoom));
        return (2.0f * half_h) / std::max(1.0f, viewport_h);
    }

    const float d = std::max(0.01f, depth);
    return 2.0f * d * std::tan(glm::radians(fov_y * 0.5f)) / std::max(1.0f, viewport_h);
}

void Camera::screen_ray(float ndc_x, float ndc_y, float viewport_width, float viewport_height,
                        glm::vec3& out_origin, glm::vec3& out_direction) const
{
    const glm::mat4 proj = get_projection(viewport_width, viewport_height);
    const glm::mat4 view = get_view();

    if (projection_type == ProjectionType::ORTHOGRAPHIC)
    {
        // Parallel projection : every pixel shares the camera forward direction.
        // The origin slides across the orthographic box at the near plane distance.
        const auto [half_w, half_h] = ortho_extents(viewport_width, viewport_height);

        const glm::vec3 forward = glm::normalize(target - position);

        glm::vec3 right = glm::cross(forward, up);
        if (glm::length(right) < 1e-4f)
        {
            // forward is parallel to up : pick any perpendicular axis
            right = glm::cross(forward, glm::vec3(0.0f, 0.0f, 1.0f));
            if (glm::length(right) < 1e-4f)
            {
                right = glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f));
            }
        }
        right = glm::normalize(right);

        const glm::vec3 cam_up = glm::normalize(glm::cross(right, forward));

        out_origin    = position + right * (ndc_x * half_w) + cam_up * (ndc_y * half_h);
        out_direction = forward;
        return;
    }

    // Perspective : unproject the near-plane point and aim from the camera position
    const glm::vec4 ray_clip(ndc_x, ndc_y, -1.0f, 1.0f);
    glm::vec4       ray_eye = glm::inverse(proj) * ray_clip;
    ray_eye.z               = -1.0f;
    ray_eye.w               = 0.0f;

    out_origin    = position;
    out_direction = glm::normalize(glm::vec3(glm::inverse(view) * ray_eye));
}

bool any_projection_differs(const MultiViewContext& ctx, int active_count)
{
    for (int i = 1; i < active_count && i < MultiViewContext::MAX_CAMERAS; ++i)
    {
        if (ctx.cameras[i].projection_type != ctx.cameras[0].projection_type)
        {
            return true;
        }
    }

    return false;
}

void Camera::rotate(double dx, double dy)
{
    glm::vec3   offset  = position - target;
    const float angle_x = static_cast<float>(-dx * 0.005);
    const float angle_y = static_cast<float>(-dy * 0.005);

    offset = glm::vec3(glm::rotate(glm::mat4(1.0f), angle_x, up) * glm::vec4(offset, 0.0f));

    const glm::vec3 forward    = glm::normalize(-offset);
    const glm::vec3 right      = glm::normalize(glm::cross(forward, up));
    const float     pitch      = glm::asin(glm::clamp(glm::dot(forward, up), -1.0f, 1.0f));
    const float     max_pitch  = glm::radians(89.0f);
    const float     pitch_step = glm::clamp(angle_y, -max_pitch - pitch, max_pitch - pitch);

    offset   = glm::vec3(glm::rotate(glm::mat4(1.0f), pitch_step, right) * glm::vec4(offset, 0.0f));
    position = target + offset;
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

    if (projection_type == ProjectionType::ORTHOGRAPHIC)
    {
        // Orthographic : there is no perspective foreshortening, so dolly is faked by
        // zooming the projection box. scroll > 0 (wheel up) magnifies, scroll < 0 zooms out.
        // Planes are left untouched, the user controls them with the sliders.
        const float factor = std::pow(0.9f, static_cast<float>(scroll));
        ortho_zoom         = std::clamp(ortho_zoom * factor, 1e-3f, 1e4f);
        return;
    }

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
        // Ctrl + right button is used for point picking, do not start panning in that case
        if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) != GLFW_PRESS)
        {
            camera.panning = true;
        }
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
