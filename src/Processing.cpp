#include <cave-traversal-tool/Processing.h>

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
    const bool                         use_centered,
    const bool                         mark_draw)
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

            bucket.draw = mark_draw;
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

std::vector<glm::ivec3> find_buckets_in_aabb(const PointCloudBucket& g_buckets, const AABB& aabb)
{
    std::vector<glm::ivec3> result;

    for (const auto& [id, bucket] : g_buckets)
    {
        const glm::vec3& bmin = bucket.aabb.min;
        const glm::vec3& bmax = bucket.aabb.max;

        bool overlap = (bmin.x <= aabb.max.x && bmax.x >= aabb.min.x) &&
                       (bmin.y <= aabb.max.y && bmax.y >= aabb.min.y) &&
                       (bmin.z <= aabb.max.z && bmax.z >= aabb.min.z);

        if (overlap)
        {
            result.push_back(id);
        }
    }

    return result;
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

void free_point_cloud_record(PointCloudRecord* record)
{
    if (!record)
    {
        return;
    }

    PointCloudLOD* current = record->lods;
    while (current)
    {
        PointCloudLOD* next = current->next;
        delete current;
        current = next;
    }

    record->lods = nullptr;
}