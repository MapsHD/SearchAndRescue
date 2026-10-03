#pragma once

#include <array>

#include <glm/glm.hpp>

struct AABB
{
    glm::vec3 min = {};
    glm::vec3 max = {};
};

struct OBB
{
    std::array<glm::vec3, 8> conrners = {};
};

// CPU-side orientation: full 3x3 rotation matrix as loaded from the trajectory CSV
struct TrajectoryPoseOrientationMat33
{
    glm::mat3 orientation = {};
};

// GPU-side orientation vertex: three tightly-packed axis columns ready for vertex attributes.
// Memory layout per vertex: [ axis_x (12 B) | axis_y (12 B) | axis_z (12 B) ] = 36 B, no padding.
// Maps to shader locations: in_AxisX @ loc 1, in_AxisY @ loc 2, in_AxisZ @ loc 3.
struct OrientationAxesVertex
{
    glm::vec3 axis_x = {}; // rotation matrix column 0 (local X / forward)
    glm::vec3 axis_y = {}; // rotation matrix column 1 (local Y / left)
    glm::vec3 axis_z = {}; // rotation matrix column 2 (local Z / up)
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

// Position + RGB colour vertex, used by geometry with per-vertex colours (measurement lines)
struct ColoredVertex
{
    glm::vec3 position = {};
    glm::vec3 color    = {};
};

struct PointIntensity
{
    glm::vec3 position  = {};
    float     intensity = {};
};
