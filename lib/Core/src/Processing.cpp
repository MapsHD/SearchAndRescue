#include <Core/Processing.h>

#include <algorithm>
#include <limits>
#include <vector>

glm::ivec3 calculate_bucket_id(const glm::vec3& p, const float E, const bool use_centered)
{
    glm::vec3 scaled = p / E;

    if (use_centered)
    {
        return glm::ivec3(
            static_cast<int>(std::floor(scaled.x + 0.5f)),
            static_cast<int>(std::floor(scaled.y + 0.5f)),
            static_cast<int>(std::floor(scaled.z + 0.5f)));
    }

    return glm::ivec3(
        static_cast<int>(std::floor(scaled.x)),
        static_cast<int>(std::floor(scaled.y)),
        static_cast<int>(std::floor(scaled.z)));
}

void decimate(const std::vector<PointIntensity>& in, std::vector<PointIntensity>& out, const size_t D)
{
    out.clear();
    if (D <= 1)
    {
        out = in;
        return;
    }

    const size_t input_size  = in.size();
    const size_t output_size = (input_size + D - 1) / D;

    out.reserve(output_size);

    for (size_t i = 0; i < input_size; i += D)
    {
        out.push_back(in[i]);
    }
}

void bucketize_point_cloud(
    const std::vector<PointIntensity>& points,
    PointCloudBucket&                  out_buckets,
    const float                        E,
    const size_t                       D,
    const size_t                       L,
    const size_t                       N,
    const bool                         use_centered)
{
    out_buckets.clear();

    for (const auto& p : points)
    {
        const glm::vec3& pos       = p.position;
        glm::ivec3       bucket_id = calculate_bucket_id(pos, E, use_centered);
        auto&            bucket    = out_buckets[bucket_id];

        if (!bucket.lods)
        {
            auto* head = new PointCloudLOD{};
            head->points.push_back(p);
            head->min = pos;
            head->max = pos;

            bucket.lods = head;

            if (use_centered)
            {
                glm::vec3 center = glm::vec3(bucket_id) * E;
                bucket.aabb.min  = center - glm::vec3(0.5f * E);
                bucket.aabb.max  = center + glm::vec3(0.5f * E);
            }
            else
            {
                bucket.aabb.min = glm::vec3(bucket_id) * E;
                bucket.aabb.max = bucket.aabb.min + glm::vec3(E);
            }

            bucket.draw = true;
        }
        else
        {
            PointCloudLOD* head = bucket.lods;
            head->points.push_back(p);
            head->min = glm::min(head->min, pos);
            head->max = glm::max(head->max, pos);
        }
    }

    for (auto it = out_buckets.begin(); it != out_buckets.end();)
    {
        PointCloudLOD* head = it->second.lods;

        if (!head || head->points.size() < N)
        {
            if (head)
            {
                delete head;
            }

            it = out_buckets.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (auto& [id, bucket] : out_buckets)
    {
        PointCloudLOD* current        = bucket.lods;
        size_t         levels_created = 1;

        while (levels_created < L)
        {
            if (!current)
            {
                break;
            }

            if (current->points.size() < 2)
            {
                break;
            }

            auto* next = new PointCloudLOD{};
            decimate(current->points, next->points, D);

            if (next->points.empty())
            {
                delete next;
                break;
            }

            next->min = next->points[0].position;
            next->max = next->min;

            for (const auto& p : next->points)
            {
                glm::vec3 v = p.position;
                next->min   = glm::min(next->min, v);
                next->max   = glm::max(next->max, v);
            }

            current->next = next;
            current       = next;
            ++levels_created;
        }
    }
}

OBB aabb_to_obb(const AABB& aabb, const glm::mat4& transform)
{
    OBB obb{};
    obb.conrners[0] = glm::vec3(transform * glm::vec4(aabb.min.x, aabb.min.y, aabb.min.z, 1.0f));
    obb.conrners[1] = glm::vec3(transform * glm::vec4(aabb.max.x, aabb.min.y, aabb.min.z, 1.0f));
    obb.conrners[2] = glm::vec3(transform * glm::vec4(aabb.max.x, aabb.max.y, aabb.min.z, 1.0f));
    obb.conrners[3] = glm::vec3(transform * glm::vec4(aabb.min.x, aabb.max.y, aabb.min.z, 1.0f));
    obb.conrners[4] = glm::vec3(transform * glm::vec4(aabb.min.x, aabb.min.y, aabb.max.z, 1.0f));
    obb.conrners[5] = glm::vec3(transform * glm::vec4(aabb.max.x, aabb.min.y, aabb.max.z, 1.0f));
    obb.conrners[6] = glm::vec3(transform * glm::vec4(aabb.max.x, aabb.max.y, aabb.max.z, 1.0f));
    obb.conrners[7] = glm::vec3(transform * glm::vec4(aabb.min.x, aabb.max.y, aabb.max.z, 1.0f));

    return obb;
}

bool point_in_obb(const glm::vec3& p, const OBB& obb)
{
    // Build an orthonormal-ish basis from the OBB edges and project the point onto it
    const glm::vec3 u = glm::normalize(obb.conrners[1] - obb.conrners[0]);
    const glm::vec3 v = glm::normalize(obb.conrners[3] - obb.conrners[0]);
    const glm::vec3 w = glm::normalize(obb.conrners[4] - obb.conrners[0]);

    float u_min = std::numeric_limits<float>::max(), u_max = -std::numeric_limits<float>::max();
    float v_min = std::numeric_limits<float>::max(), v_max = -std::numeric_limits<float>::max();
    float w_min = std::numeric_limits<float>::max(), w_max = -std::numeric_limits<float>::max();

    for (int i = 0; i < 8; ++i)
    {
        const glm::vec3& c = obb.conrners[i];

        const float pu = glm::dot(c, u);
        const float pv = glm::dot(c, v);
        const float pw = glm::dot(c, w);

        u_min = std::min(u_min, pu);
        u_max = std::max(u_max, pu);
        v_min = std::min(v_min, pv);
        v_max = std::max(v_max, pv);
        w_min = std::min(w_min, pw);
        w_max = std::max(w_max, pw);
    }

    const float pu = glm::dot(p, u);
    const float pv = glm::dot(p, v);
    const float pw = glm::dot(p, w);

    return (pu >= u_min && pu <= u_max) &&
           (pv >= v_min && pv <= v_max) &&
           (pw >= w_min && pw <= w_max);
}

std::pair<std::vector<glm::ivec3>, std::vector<glm::ivec3>> find_buckets_in_obb(const PointCloudBucket& g_buckets, const OBB& obb, const float M)
{
    std::vector<glm::ivec3> intersecting;
    std::vector<glm::ivec3> proximity;

    glm::vec3 obb_center = {};
    for (int i = 0; i < 8; ++i)
    {
        obb_center += obb.conrners[i];
    }
    obb_center /= 8.0f;

    for (const auto& [id, bucket] : g_buckets)
    {
        glm::vec3 aabb_center = 0.5f * (bucket.aabb.min + bucket.aabb.max);
        glm::vec3 aabb_half   = 0.5f * (bucket.aabb.max - bucket.aabb.min);

        bool overlap = true;

        glm::vec3 axes[15];
        axes[0] = {1.0f, 0.0f, 0.0f};
        axes[1] = {0.0f, 1.0f, 0.0f};
        axes[2] = {0.0f, 0.0f, 1.0f};

        axes[3] = glm::normalize(obb.conrners[1] - obb.conrners[0]);
        axes[4] = glm::normalize(obb.conrners[3] - obb.conrners[0]);
        axes[5] = glm::normalize(obb.conrners[4] - obb.conrners[0]);

        int axis_count = 6;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 3; j < 6; ++j)
            {
                axes[axis_count++] = glm::cross(axes[i], axes[j]);
            }
        }

        for (int i = 0; i < axis_count; ++i)
        {
            glm::vec3 axis = axes[i];

            if (glm::dot(axis, axis) < 1e-6f)
            {
                continue;
            }

            axis = glm::normalize(axis);

            float obb_min = std::numeric_limits<float>::max();
            float obb_max = -std::numeric_limits<float>::max();

            for (int c = 0; c < 8; ++c)
            {
                float p = glm::dot(obb.conrners[c], axis);
                obb_min = std::min(obb_min, p);
                obb_max = std::max(obb_max, p);
            }

            float aabb_center_proj = glm::dot(aabb_center, axis);
            float aabb_radius =
                aabb_half.x * std::abs(axis.x) +
                aabb_half.y * std::abs(axis.y) +
                aabb_half.z * std::abs(axis.z);

            float aabb_min = aabb_center_proj - aabb_radius;
            float aabb_max = aabb_center_proj + aabb_radius;

            if (obb_max < aabb_min || aabb_max < obb_min)
            {
                overlap = false;
                break;
            }
        }

        if (overlap)
        {
            intersecting.push_back(id);
            continue;
        }

        float dist = glm::length(aabb_center - obb_center);
        if (dist <= M)
        {
            proximity.push_back(id);
        }
    }

    return {intersecting, proximity};
}

size_t lod_from_distance(const float distance, const float max_distance, const size_t lod_count)
{
    if (lod_count == 0)
    {
        return 0;
    }

    float t = glm::clamp(distance / max_distance, 0.0f, 1.0f);

    return static_cast<size_t>(t * static_cast<float>(lod_count - 1));
}

PointCloudLOD* get_lod_at_index(PointCloudRecord* record, const size_t index)
{
    if (!record || !record->lods)
    {
        return nullptr;
    }

    size_t         i   = 0;
    PointCloudLOD* lod = record->lods;

    while (lod && i < index)
    {
        lod = lod->next;
        ++i;
    }

    return lod ? lod : record->lods;
}

void compute_camera_frustum_planes(const glm::mat4& view, const glm::mat4& projection, std::array<glm::vec4, 6>& out_planes)
{
    glm::mat4 VP = projection * view;
    glm::mat4 M  = glm::transpose(VP);

    out_planes[0] = M[3] + M[0];
    out_planes[1] = M[3] - M[0];
    out_planes[2] = M[3] + M[1];
    out_planes[3] = M[3] - M[1];
    out_planes[4] = M[3] + M[2];
    out_planes[5] = M[3] - M[2];

    for (int i = 0; i < 6; ++i)
    {
        float len = glm::length(glm::vec3(out_planes[i]));
        out_planes[i] /= len;
    }
}

bool lod_in_camera_frustum(const PointCloudLOD& lod, const std::array<glm::vec4, 6>& planes)
{
    const glm::vec3& bmin = lod.min;
    const glm::vec3& bmax = lod.max;

    for (int i = 0; i < 6; ++i)
    {
        const glm::vec4& p = planes[i];

        glm::vec3 positive{
            (p.x >= 0.0f) ? bmax.x : bmin.x,
            (p.y >= 0.0f) ? bmax.y : bmin.y,
            (p.z >= 0.0f) ? bmax.z : bmin.z};

        if (glm::dot(glm::vec3(p), positive) + p.w < 0.0f)
        {
            return false;
        }
    }

    return true;
}

bool record_in_camera_frustum(const PointCloudRecord& record, const std::array<glm::vec4, 6>& planes)
{
    const glm::vec3& bmin = record.aabb.min;
    const glm::vec3& bmax = record.aabb.max;

    for (int i = 0; i < 6; ++i)
    {
        const glm::vec4& p = planes[i];

        glm::vec3 positive{
            (p.x >= 0.0f) ? bmax.x : bmin.x,
            (p.y >= 0.0f) ? bmax.y : bmin.y,
            (p.z >= 0.0f) ? bmax.z : bmin.z};

        if (glm::dot(glm::vec3(p), positive) + p.w < 0.0f)
        {
            return false;
        }
    }

    return true;
}

bool bucket_in_front_of_camera(const AABB& aabb, const glm::vec4& near_plane, const glm::vec3& camera_pos)
{
    const glm::vec3& bmin = aabb.min;
    const glm::vec3& bmax = aabb.max;

    // Positive vertex : the AABB corner that is furthest along the near plane normal.
    // If even this corner sits behind the plane, the whole box does.
    const glm::vec3 positive{
        (near_plane.x >= 0.0f) ? bmax.x : bmin.x,
        (near_plane.y >= 0.0f) ? bmax.y : bmin.y,
        (near_plane.z >= 0.0f) ? bmax.z : bmin.z};

    if (glm::dot(glm::vec3(near_plane), positive) + near_plane.w < 0.0f)
    {
        return false;
    }

    // Orthographic cameras : the near plane is parallel to the image plane, so its signed distance
    // is exactly the depth behind the camera (positive in front). The conservative positive vertex
    // test above is too weak there (a large bucket can have its furthest corner in front while
    // sitting entirely behind the camera), so use the box centre.
    const glm::vec3 center = 0.5f * (bmin + bmax);
    if (glm::dot(glm::vec3(near_plane), center - camera_pos) + near_plane.w < 0.0f)
    {
        return false;
    }

    return true;
}

namespace
{
    // Slab test of a ray against an AABB : returns the ray parameters of the entry and exit points
    // (the entry may be negative when the ray origin is inside the box)
    bool intersect_ray_aabb(const glm::vec3& origin, const glm::vec3& direction, const glm::vec3& box_min, const glm::vec3& box_max, float& out_t_enter, float& out_t_exit)
    {
        float t_enter = -std::numeric_limits<float>::max();
        float t_exit  = std::numeric_limits<float>::max();

        for (int i = 0; i < 3; ++i)
        {
            if (std::abs(direction[i]) < 1e-6f)
            {
                if (origin[i] < box_min[i] || origin[i] > box_max[i])
                {
                    return false;
                }
                continue;
            }

            const float inv_d = 1.0f / direction[i];
            float       t0    = (box_min[i] - origin[i]) * inv_d;
            float       t1    = (box_max[i] - origin[i]) * inv_d;

            if (t0 > t1)
            {
                std::swap(t0, t1);
            }

            t_enter = std::max(t_enter, t0);
            t_exit  = std::min(t_exit, t1);

            if (t_enter > t_exit)
            {
                return false;
            }
        }

        out_t_enter = t_enter;
        out_t_exit  = t_exit;
        return true;
    }

    // View state and tolerance shared by the ray pick queries
    struct RayPickFilter
    {
        const Camera&             camera;
        const PointPickTolerance& tolerance;

        // Depth is measured along the camera forward axis from the camera position, the same way the near / far
        // planes clip what is rendered. It must not be the distance along the ray : for a perspective camera an
        // off-centre ray is longer than the view depth, so visible points close to the far plane would be rejected
        // near the viewport edges (this shows up in the narrow near / far slab of the locked axis viewports).
        // For an orthographic camera both are identical, the ray origin only shifts perpendicular to the forward axis.
        bool      is_ortho;
        glm::vec3 view_forward;

        // World units per screen pixel : proportional to the depth for perspective, constant for orthographic
        float world_per_pixel_at_unit_depth;

        RayPickFilter(const Camera& in_camera, const PointPickTolerance& in_tolerance)
            : camera(in_camera),
              tolerance(in_tolerance),
              is_ortho(in_camera.projection_type == ProjectionType::PROJECTION_TYPE_ORTHOGRAPHIC),
              view_forward(glm::normalize(in_camera.target - in_camera.position)),
              world_per_pixel_at_unit_depth(in_camera.world_units_per_pixel(1.0f))
        {
        }

        float depth_of(const glm::vec3& position) const
        {
            return glm::dot(position - camera.position, view_forward);
        }

        float radius_at_depth(const float depth) const
        {
            switch (tolerance.mode)
            {
            case PickToleranceMode::PICK_TOLERANCE_MODE_WORLD:
                return tolerance.radius_m;
            case PickToleranceMode::PICK_TOLERANCE_MODE_SCREEN:
                return screen_radius_at_depth(depth);
            case PickToleranceMode::PICK_TOLERANCE_MODE_LARGEST:
                break;
            }
            return std::max(tolerance.radius_m, screen_radius_at_depth(depth));
        }

        float screen_radius_at_depth(const float depth) const
        {
            return tolerance.radius_px * world_per_pixel_at_unit_depth * (is_ortho ? 1.0f : std::max(depth, 0.0f));
        }

        // A position is pickable when it is in front of a perspective ray origin, between the camera near / far
        // planes and within the tolerance of the ray. Positions further along the ray than max_t are skipped.
        // out_t is the distance along the ray, out_distance the perpendicular distance to the ray.
        bool test(const glm::vec3& position, const glm::vec3& ray_origin, const glm::vec3& ray_direction, const float max_t, float& out_t, float& out_distance) const
        {
            const glm::vec3 to_point = position - ray_origin;
            const float     t        = glm::dot(to_point, ray_direction);

            if (t >= max_t || (!is_ortho && t <= 0.0f))
            {
                return false;
            }

            const float depth = depth_of(position);

            if (depth < camera.near_plane || depth > camera.far_plane)
            {
                return false;
            }

            const float     radius  = radius_at_depth(depth);
            const glm::vec3 off_ray = to_point - t * ray_direction;
            const float     dist_sq = glm::dot(off_ray, off_ray);

            if (dist_sq > radius * radius)
            {
                return false;
            }

            out_t        = t;
            out_distance = std::sqrt(dist_sq);
            return true;
        }
    };
} // namespace

std::optional<glm::vec3> pick_point_along_ray(
    const PointCloudBucket&   buckets,
    const Camera&             camera,
    const glm::vec3&          ray_origin,
    const glm::vec3&          ray_direction,
    const PointPickTolerance& tolerance)
{
    const RayPickFilter filter(camera, tolerance);
    const glm::vec3     abs_forward = glm::abs(filter.view_forward);

    struct BucketCandidate
    {
        float                   t_enter;
        const PointCloudRecord* bucket;
    };

    std::vector<BucketCandidate> candidates;
    candidates.reserve(buckets.size());

    for (const auto& [id, bucket] : buckets)
    {
        if (!bucket.lods)
        {
            continue;
        }

        const glm::vec3 center       = (bucket.aabb.min + bucket.aabb.max) * 0.5f;
        const glm::vec3 half_extent  = (bucket.aabb.max - bucket.aabb.min) * 0.5f;
        const float     center_depth = filter.depth_of(center);
        const float     reach        = glm::dot(abs_forward, half_extent);

        // The ray may hit a point of the bucket without crossing its box, grow the box by the pick radius
        const float margin = filter.radius_at_depth(center_depth + reach);

        // Everything outside the near / far planes is not rendered, so it is not pickable either
        if (center_depth + reach + margin < camera.near_plane || center_depth - reach - margin > camera.far_plane)
        {
            continue;
        }

        float t_enter = 0.0f;
        float t_exit  = 0.0f;
        if (!intersect_ray_aabb(ray_origin, ray_direction, bucket.aabb.min - margin, bucket.aabb.max + margin, t_enter, t_exit))
        {
            continue;
        }

        // A perspective ray starts at the camera, a box that ends before the ray origin is behind the camera.
        // An orthographic ray origin sits on the camera plane, the box may legitimately lie behind it.
        if (!filter.is_ortho && t_exit < 0.0f)
        {
            continue;
        }

        candidates.push_back({t_enter, &bucket});
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const BucketCandidate& a, const BucketCandidate& b)
              { return a.t_enter < b.t_enter; });

    std::optional<glm::vec3> best_point{};
    float                    best_t = std::numeric_limits<float>::max();

    for (const BucketCandidate& candidate : candidates)
    {
        // Buckets are sorted by the distance at which the ray reaches them : a bucket entered beyond the best
        // point found so far cannot contain anything closer to the camera
        if (candidate.t_enter >= best_t)
        {
            break;
        }

        for (const PointCloudLOD* lod = candidate.bucket->lods; lod; lod = lod->next)
        {
            for (const PointIntensity& p : lod->points)
            {
                float t        = 0.0f;
                float distance = 0.0f;
                if (filter.test(p.position, ray_origin, ray_direction, best_t, t, distance))
                {
                    best_point = p.position;
                    best_t     = t;
                }
            }
        }
    }

    return best_point;
}

std::optional<size_t> pick_trajectory_point_along_ray(
    const std::vector<Point>& trajectory,
    const Camera&             camera,
    const glm::vec3&          ray_origin,
    const glm::vec3&          ray_direction,
    const PointPickTolerance& tolerance)
{
    const RayPickFilter filter(camera, tolerance);

    std::optional<size_t> best_index{};
    float                 best_distance = std::numeric_limits<float>::max();
    float                 best_t        = std::numeric_limits<float>::max();

    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        float t        = 0.0f;
        float distance = 0.0f;
        if (!filter.test(trajectory[i].position, ray_origin, ray_direction, std::numeric_limits<float>::max(), t, distance))
        {
            continue;
        }

        // Trajectory poses are sparse, prefer the one closest to the ray, then the one closest to the camera
        if (!best_index || distance < best_distance - 1e-4f || (distance < best_distance + 1e-4f && t < best_t))
        {
            best_index    = i;
            best_distance = distance;
            best_t        = t;
        }
    }

    return best_index;
}
