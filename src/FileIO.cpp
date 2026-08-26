#include <cave-traversal-tool/FileIO.h>

#include <happly.h>
#include <spdlog/spdlog.h>

bool load_text_file(std::vector<char>& output, const std::filesystem::path& path)
{
    if (std::ifstream file = std::ifstream(path, std::ios::binary))
    {
        file.seekg(0, std::ios::end);
        const size_t size = static_cast<size_t>(file.tellg());
        file.seekg(0, std::ios::beg);

        output.resize(size + 1UL);
        file.read(output.data(), size);
        output[size] = 0x00;

        return true;
    }

    return false;
}

bool load_trajectory_csv_mat33(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth)
{
    struct CSVTrajectoryPoseMat33
    {
        uint64_t ts_lidar = 0ULL;
        uint64_t ts_linux = 0ULL;

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        float r0 = 0.0f;
        float r1 = 0.0f;
        float r2 = 0.0f;

        float r3 = 0.0f;
        float r4 = 0.0f;
        float r5 = 0.0f;

        float r6 = 0.0f;
        float r7 = 0.0f;
        float r8 = 0.0f;
    };

    trajectory_pose_positions    = {};
    trajectory_pose_orientations = {};

    if (std::ifstream file = std::ifstream(path))
    {
        std::vector<CSVTrajectoryPoseMat33> csv_poses{};

        size_t      line_index = 0;
        std::string line{};
        std::string token{};

        while (std::getline(file, line))
        {
            if (line.empty())
                continue;

            if (line_index % Nth != 0)
            {
                line_index++;
                continue;
            }

            std::stringstream      string_stream(line);
            CSVTrajectoryPoseMat33 csv_pose{};

            std::getline(string_stream, token, ',');
            csv_pose.ts_lidar = std::stoull(token);

            std::getline(string_stream, token, ',');
            csv_pose.ts_linux = std::stoull(token);

            std::getline(string_stream, token, ',');
            csv_pose.x = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.y = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.z = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r0 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r1 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r2 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r3 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r4 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r5 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r6 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r7 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r8 = std::stof(token);

            csv_poses.push_back(csv_pose);
            line_index++;
        }

        trajectory_pose_positions    = {};
        trajectory_pose_orientations = {};

        for (const auto& csv_pose : csv_poses)
        {
            Point point{};
            point.position.x = csv_pose.x;
            point.position.y = csv_pose.y;
            point.position.z = csv_pose.z;

            TrajectoryPoseOrientationMat33 orientation{};
            orientation.orientation = glm::mat3(
                csv_pose.r0, csv_pose.r1, csv_pose.r2,
                csv_pose.r3, csv_pose.r4, csv_pose.r5,
                csv_pose.r6, csv_pose.r7, csv_pose.r8);

            trajectory_pose_positions.push_back(point);
            trajectory_pose_orientations.push_back(orientation);
        }

        return true;
    }

    return false;
}

bool load_trajectory_bin_mat33(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth)
{
    trajectory_pose_positions    = {};
    trajectory_pose_orientations = {};

    if (std::ifstream file = std::ifstream(path, std::ios::in | std::ios::binary))
    {
        uint32_t trajectory_length = 0;
        file.read(reinterpret_cast<char*>(&trajectory_length), sizeof(uint32_t));

        std::vector<Point> all_positions(trajectory_length);
        file.read(reinterpret_cast<char*>(all_positions.data()), trajectory_length * sizeof(Point));

        std::vector<TrajectoryPoseOrientationMat33> all_orientations(trajectory_length);
        file.read(reinterpret_cast<char*>(all_orientations.data()), trajectory_length * sizeof(TrajectoryPoseOrientationMat33));

        size_t index = 0;
        for (uint32_t i = 0; i < trajectory_length; ++i)
        {
            if (Nth > 1 && (index % Nth) != 0)
            {
                ++index;
                continue;
            }

            trajectory_pose_positions.push_back(all_positions[i]);
            trajectory_pose_orientations.push_back(all_orientations[i]);
            ++index;
        }

        return true;
    }

    return false;
}

bool save_trajectory_bin_mat33(const std::filesystem::path& path, const std::vector<Point>& trajectory_pose_positions, const std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations)
{
    if (std::ofstream file = std::ofstream(path, std::ios::out | std::ios::binary))
    {
        const uint32_t trajectory_length = static_cast<uint32_t>(trajectory_pose_positions.size());

        const size_t positions_size          = trajectory_length * sizeof(Point);
        const size_t orientations_mat33_size = trajectory_length * sizeof(TrajectoryPoseOrientationMat33);

        const Point* const                          positions          = trajectory_pose_positions.data();
        const TrajectoryPoseOrientationMat33* const orientations_mat33 = trajectory_pose_orientations.data();

        file.write(reinterpret_cast<const char* const>(&trajectory_length), sizeof(uint32_t));
        file.write(reinterpret_cast<const char* const>(positions), positions_size);
        file.write(reinterpret_cast<const char* const>(orientations_mat33), orientations_mat33_size);

        return true;
    }

    return false;
}

bool load_stretcher_ply(const std::filesystem::path& path, std::vector<ColorPoint>& points, std::vector<uint32_t>& indices)
{
    points  = {};
    indices = {};

    happly::PLYData ply(path.string());

    if (!ply.hasElement("vertex") || ply.getVertexPositions().empty())
    {
        spdlog::error("PLY file {} has no vertex positions!", path.string());
        return false;
    }

    const auto& vertexNames = ply.getElement("vertex").getPropertyNames();
    if (!(std::count(vertexNames.begin(), vertexNames.end(), "red") &&
          std::count(vertexNames.begin(), vertexNames.end(), "green") &&
          std::count(vertexNames.begin(), vertexNames.end(), "blue")))
    {
        spdlog::error("PLY file {} has no vertex colors!", path.string());
        return false;
    }

    if (!ply.hasElement("face") || ply.getFaceIndices().empty())
    {
        spdlog::error("PLY file {} has no face indices!", path.string());
        return false;
    }

    const auto positions = ply.getVertexPositions();
    const auto colors    = ply.getVertexColors();

    points.clear();
    points.reserve(positions.size());

    for (size_t i = 0; i < positions.size(); i++)
    {
        ColorPoint pt;
        pt.position.x = static_cast<float>(positions[i][0]);
        pt.position.y = static_cast<float>(positions[i][1]);
        pt.position.z = static_cast<float>(positions[i][2]);

        pt.color.x = colors[i][0];
        pt.color.y = colors[i][1];
        pt.color.z = colors[i][2];

        points.push_back(pt);
    }

    const auto faceIndices = ply.getFaceIndices();

    indices.clear();

    for (const auto& f : faceIndices)
    {
        if (f.size() != 3)
        {
            spdlog::error("PLY file {} contains a non-triangle face!", path.string());
            return false;
        }

        indices.push_back(f[0]);
        indices.push_back(f[1]);
        indices.push_back(f[2]);
    }

    return true;
}

bool load_stretcher_bin(const std::filesystem::path& path, std::vector<ColorPoint>& points, std::vector<uint32_t>& indices)
{
    points  = {};
    indices = {};

    if (std::ifstream file = std::ifstream(path, std::ios::in | std::ios::binary))
    {
        points  = {};
        indices = {};

        uint32_t vertices_count = 0;
        uint32_t indices_count  = 0;

        file.read(reinterpret_cast<char*>(&vertices_count), sizeof(uint32_t));
        file.read(reinterpret_cast<char*>(&indices_count), sizeof(uint32_t));

        points.resize(vertices_count);
        indices.resize(indices_count);

        file.read(reinterpret_cast<char*>(points.data()), vertices_count * sizeof(ColorPoint));
        file.read(reinterpret_cast<char*>(indices.data()), indices_count * sizeof(uint32_t));

        return true;
    }

    return false;
}

bool save_stretcher_bin(const std::filesystem::path& path, const std::vector<ColorPoint>& points, const std::vector<uint32_t>& indices)
{
    if (std::ofstream file = std::ofstream(path, std::ios::out | std::ios::binary))
    {
        const uint32_t stretcher_vertices_count = points.size();
        const uint32_t stretcher_indices_count  = indices.size();

        const ColorPoint* const stretcher_vertices = points.data();
        const uint32_t* const   stretcher_indices  = indices.data();

        file.write(reinterpret_cast<const char* const>(&stretcher_vertices_count), sizeof(uint32_t));
        file.write(reinterpret_cast<const char* const>(&stretcher_indices_count), sizeof(uint32_t));
        file.write(reinterpret_cast<const char* const>(stretcher_vertices), stretcher_vertices_count * sizeof(ColorPoint));
        file.write(reinterpret_cast<const char* const>(stretcher_indices), stretcher_indices_count * sizeof(uint32_t));

        return true;
    }

    return false;
}

bool load_cave_ply(const std::filesystem::path& path, std::vector<NormalPoint>& points)
{
    points = {};

    happly::PLYData ply(path.string());

    if (!ply.hasElement("vertex") || ply.getVertexPositions().empty())
    {
        spdlog::error("PLY file {} has no vertex positions!", path.string());
        return false;
    }

    const auto& vertexNames = ply.getElement("vertex").getPropertyNames();
    if (!(std::count(vertexNames.begin(), vertexNames.end(), "nx") &&
          std::count(vertexNames.begin(), vertexNames.end(), "ny") &&
          std::count(vertexNames.begin(), vertexNames.end(), "nz")))
    {
        spdlog::error("PLY file {} has no vertex normals!", path.string());
        return false;
    }

    const auto positions = ply.getVertexPositions();

    const auto nx = ply.getElement("vertex").getProperty<float>("nx");
    const auto ny = ply.getElement("vertex").getProperty<float>("ny");
    const auto nz = ply.getElement("vertex").getProperty<float>("nz");

    points.clear();
    points.reserve(positions.size());

    for (size_t i = 0; i < positions.size(); i++)
    {
        NormalPoint pt;

        pt.position.x = static_cast<float>(positions[i][0]);
        pt.position.y = static_cast<float>(positions[i][1]);
        pt.position.z = static_cast<float>(positions[i][2]);

        pt.normal.x = nx[i];
        pt.normal.y = ny[i];
        pt.normal.z = nz[i];

        points.push_back(pt);
    }

    return true;
}

bool load_cave_bin(const std::filesystem::path& path, std::vector<NormalPoint>& points)
{
    points = {};

    if (std::ifstream file = std::ifstream(path, std::ios::in | std::ios::binary))
    {
        uint32_t vertices_count = 0;
        file.read(reinterpret_cast<char*>(&vertices_count), sizeof(uint32_t));

        points.resize(vertices_count);
        file.read(reinterpret_cast<char*>(points.data()), vertices_count * sizeof(NormalPoint));

        return true;
    }

    return false;
}

bool save_cave_bin(const std::filesystem::path& path, const std::vector<NormalPoint>& points)
{
    if (std::ofstream file = std::ofstream(path, std::ios::out | std::ios::binary))
    {
        const uint32_t           vertices_count = points.size();
        const NormalPoint* const vertices       = points.data();

        file.write(reinterpret_cast<const char* const>(&vertices_count), sizeof(uint32_t));
        file.write(reinterpret_cast<const char* const>(vertices), vertices_count * sizeof(NormalPoint));

        return true;
    }

    return false;
}
