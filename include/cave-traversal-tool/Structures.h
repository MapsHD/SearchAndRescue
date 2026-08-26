#pragma once

#include <array>

#include <glm/glm.hpp>

#include <unordered_map>

struct AABB
{
    glm::vec3 min = {};
    glm::vec3 max = {};
};

struct OBB
{
    std::array<glm::vec3, 8> conrners = {};
};

struct TrajectoryPoseOrientationMat33
{
    glm::mat3 orientation = {};
};

struct ColorPoint
{
    glm::vec3            position = {};
    glm::vec<3, uint8_t> color    = {};
};

struct Point
{
    glm::vec3 position = {};
};

struct NormalPoint
{
    glm::vec3 position = {};
    glm::vec3 normal   = {};
};

struct PointCloudLOD
{
    std::vector<NormalPoint> points;

    glm::vec3 min = {};
    glm::vec3 max = {};

    uint32_t vbo = UINT32_MAX;
    uint32_t vao = UINT32_MAX;

    PointCloudLOD* next = nullptr;
};

struct PointCloudRecord
{
    bool draw = false;

    AABB aabb = {};

    PointCloudLOD* lods = nullptr;

    uint32_t bbox_vbo = UINT32_MAX;
    uint32_t bbox_vao = UINT32_MAX;
};

struct ivec3_hash_operator
{
    size_t operator()(const glm::ivec3& v) const noexcept
    {
        const size_t h1 = std::hash<int>{}(v.x);
        const size_t h2 = std::hash<int>{}(v.y);
        const size_t h3 = std::hash<int>{}(v.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

using PointCloudBucket = std::unordered_map<glm::ivec3, PointCloudRecord, ivec3_hash_operator>;