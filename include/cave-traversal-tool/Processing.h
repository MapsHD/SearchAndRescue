#pragma once

#include <cave-traversal-tool/PointCloud.h>

glm::ivec3 calculate_bucket_id(const glm::vec3& p, const float E, const bool use_centered);

void decimate(const std::vector<PointIntensity>& in, std::vector<PointIntensity>& out, const size_t D);

void bucketize_point_cloud(
    const std::vector<PointIntensity>& points,
    PointCloudBucket&                  out_buckets,
    const float                        E,
    const size_t                       D,
    const size_t                       L,
    const size_t                       N,
    const bool                         use_centered,
    const bool                         mark_draw);

OBB aabb_to_obb(const AABB& aabb, const glm::mat4& transform);

std::vector<glm::ivec3> find_buckets_in_aabb(const PointCloudBucket& g_buckets, const AABB& aabb);

std::pair<std::vector<glm::ivec3>, std::vector<glm::ivec3>> find_buckets_in_obb(const PointCloudBucket& g_buckets, const OBB& obb, const float M);

size_t lod_from_distance(const float distance, const float max_distance, const size_t lod_count);

PointCloudLOD* get_lod_at_index(PointCloudRecord* record, const size_t index);

void compute_camera_frustum_planes(const glm::mat4& view, const glm::mat4& projection, std::array<glm::vec4, 6>& out_planes);

bool lod_in_camera_frustum(const PointCloudLOD& lod, const std::array<glm::vec4, 6>& planes);

bool record_in_camera_frustum(const PointCloudRecord& record, const std::array<glm::vec4, 6>& planes);

void free_point_cloud_record(PointCloudRecord* record);