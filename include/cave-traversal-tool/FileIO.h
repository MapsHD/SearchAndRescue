#pragma once

#include <filesystem>
#include <fstream>
#include <vector>

#include <cave-traversal-tool/Structures.h>

bool load_trajectory_csv(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth = 1);
bool load_stretcher_ply(const std::filesystem::path& path, std::vector<ColorPoint>& points, std::vector<uint32_t>& indices);
bool load_cave_laz(const std::filesystem::path& path, std::vector<PointIntensity>& points);