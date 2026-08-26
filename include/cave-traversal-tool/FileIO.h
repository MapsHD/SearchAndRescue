#pragma once

#include <filesystem>
#include <fstream>
#include <vector>

#include <cave-traversal-tool/Structures.h>

template <typename T>
static inline bool load_binary(std::vector<T>& output, const std::filesystem::path& path)
{
    if (std::ifstream file = std::ifstream(path, std::ios::ate | std::ios::binary))
    {
        const auto size = static_cast<size_t>(file.tellg());
        output.resize(size / sizeof(T));
        file.seekg(0, std::ios::beg);
        file.read((char*)output.data(), size);

        return true;
    }

    return false;
}

bool load_text_file(std::vector<char>& output, const std::filesystem::path& path);

bool load_trajectory_csv_mat33(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth = 1);
bool load_trajectory_bin_mat33(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth = 1);
bool save_trajectory_bin_mat33(const std::filesystem::path& path, const std::vector<Point>& trajectory_pose_positions, const std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations);

bool load_stretcher_ply(const std::filesystem::path& path, std::vector<ColorPoint>& points, std::vector<uint32_t>& indices);
bool load_stretcher_bin(const std::filesystem::path& path, std::vector<ColorPoint>& points, std::vector<uint32_t>& indices);
bool save_stretcher_bin(const std::filesystem::path& path, const std::vector<ColorPoint>& points, const std::vector<uint32_t>& indices);

bool load_cave_ply(const std::filesystem::path& path, std::vector<NormalPoint>& points);
bool load_cave_bin(const std::filesystem::path& path, std::vector<NormalPoint>& points);
bool save_cave_bin(const std::filesystem::path& path, const std::vector<NormalPoint>& points);