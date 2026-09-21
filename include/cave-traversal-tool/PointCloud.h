#pragma once

#include "Structures.h"

#include "OpenGL/Buffer.h"
#include "OpenGL/VertexArray.h"

struct PointCloudLOD
{
    std::vector<PointIntensity> points;

    glm::vec3 min = {};
    glm::vec3 max = {};

    Buffer*      vbo = nullptr;
    VertexArray* vao = nullptr;

    PointCloudLOD* next = nullptr;
};

struct PointCloudRecord
{
    bool draw = false;

    AABB aabb = {};

    PointCloudLOD* lods = nullptr;

    Buffer*      bbox_vbo = nullptr;
    VertexArray* bbox_vao = nullptr;
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