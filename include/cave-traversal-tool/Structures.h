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

struct PointIntensity
{
    glm::vec3 position  = {};
    float     intensity = {};
};

struct NormalPoint
{
    glm::vec3 position = {};
    glm::vec3 normal   = {};
};
