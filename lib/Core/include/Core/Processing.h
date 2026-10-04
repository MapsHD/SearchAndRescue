#pragma once

#include <Core/Camera.h>
#include <Core/PointCloud.h>

#include <optional>

glm::ivec3 calculate_bucket_id(const glm::vec3& p, const float E, const bool use_centered);

void decimate(const std::vector<PointIntensity>& in, std::vector<PointIntensity>& out, const size_t D);

void bucketize_point_cloud(
    const std::vector<PointIntensity>& points,
    PointCloudBucket&                  out_buckets,
    const float                        E,
    const size_t                       D,
    const size_t                       L,
    const size_t                       N,
    const bool                         use_centered);

OBB aabb_to_obb(const AABB& aabb, const glm::mat4& transform);

bool point_in_obb(const glm::vec3& p, const OBB& obb);

std::pair<std::vector<glm::ivec3>, std::vector<glm::ivec3>> find_buckets_in_obb(const PointCloudBucket& g_buckets, const OBB& obb, const float M);

size_t lod_from_distance(const float distance, const float max_distance, const size_t lod_count);

PointCloudLOD* get_lod_at_index(PointCloudRecord* record, const size_t index);

void compute_camera_frustum_planes(const glm::mat4& view, const glm::mat4& projection, std::array<glm::vec4, 6>& out_planes);

bool lod_in_camera_frustum(const PointCloudLOD& lod, const std::array<glm::vec4, 6>& planes);

bool record_in_camera_frustum(const PointCloudRecord& record, const std::array<glm::vec4, 6>& planes);

// Tolerance of point picking : perpendicular distance between a point and the picking ray
struct PointPickTolerance
{
    // Fixed world-space radius, also the lower bound of the on-screen tolerance
    float radius_m = 0.025f;

    // Optional on-screen tolerance in pixels, converted to world units at the view depth of each point
    // (0 disables it, only radius_m applies)
    float radius_px = 0.0f;
};

// Picks the point cloud point under the ray cast from the camera (see Camera::screen_ray).
//
// Buckets outside the camera near / far planes (this includes everything behind a perspective camera) are
// rejected, the remaining buckets crossed by the ray are visited from the closest to the furthest and
// searched for points within the tolerance of the ray. The point closest to the camera along the ray wins.
// Returns std::nullopt when no point is found.
std::optional<glm::vec3> pick_point_along_ray(
    const PointCloudBucket&   buckets,
    const Camera&             camera,
    const glm::vec3&          ray_origin,
    const glm::vec3&          ray_direction,
    const PointPickTolerance& tolerance = {});

// Same query for the trajectory : picks the trajectory point under the ray and returns its index.
// The same near / far rejection and tolerance apply, but trajectory points are sparse, so the point closest to the
// ray wins (the one closest to the camera among equals) instead of the one closest to the camera.
std::optional<size_t> pick_trajectory_point_along_ray(
    const std::vector<Point>& trajectory,
    const Camera&             camera,
    const glm::vec3&          ray_origin,
    const glm::vec3&          ray_direction,
    const PointPickTolerance& tolerance = {});
