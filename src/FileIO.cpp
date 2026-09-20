#include <cave-traversal-tool/FileIO.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <happly.h>
#include <laszip_api.h>
#include <limits>
#include <spdlog/spdlog.h>
#include <sstream>

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

enum class TrajectoryCsvLayout
{
    Unknown,
    Mat33_2Timestamps, // TS1, TS2, x, y, z, r00..r22  (14 columns)
    Mat33_1Timestamp,  // TS1,      x, y, z, r00..r22  (13 columns)
    Quat_2Timestamps,  // TS1, TS2, x, y, z, qx,qy,qz,qw (9 columns)
    Quat_1Timestamp,   // TS1,      x, y, z, qx,qy,qz,qw (8 columns)
};

static size_t count_csv_columns(const std::string& line)
{
    size_t count = line.empty() ? 0UL : 1UL;

    for (const char c : line)
        if (c == ',')
            count++;

    return count;
}

static TrajectoryCsvLayout classify_trajectory_csv_layout(const size_t column_count)
{
    switch (column_count)
    {
    case 14:
        return TrajectoryCsvLayout::Mat33_2Timestamps;
    case 13:
        return TrajectoryCsvLayout::Mat33_1Timestamp;
    case 9:
        return TrajectoryCsvLayout::Quat_2Timestamps;
    case 8:
        return TrajectoryCsvLayout::Quat_1Timestamp;
    default:
        return TrajectoryCsvLayout::Unknown;
    }
}

bool load_trajectory_csv(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth)
{
    trajectory_pose_positions    = {};
    trajectory_pose_orientations = {};

    if (std::ifstream file = std::ifstream(path))
    {
        TrajectoryCsvLayout layout = TrajectoryCsvLayout::Unknown;

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

            if (layout == TrajectoryCsvLayout::Unknown)
            {
                layout = classify_trajectory_csv_layout(count_csv_columns(line));

                if (layout == TrajectoryCsvLayout::Unknown)
                {
                    spdlog::error("Trajectory CSV {} has an unsupported column count ({}); expected 8, 9, 13 or 14 columns", path.string(), count_csv_columns(line));
                    return false;
                }
            }

            const bool is_quaternion      = (layout == TrajectoryCsvLayout::Quat_2Timestamps || layout == TrajectoryCsvLayout::Quat_1Timestamp);
            const bool has_two_timestamps = (layout == TrajectoryCsvLayout::Mat33_2Timestamps || layout == TrajectoryCsvLayout::Quat_2Timestamps);

            try
            {
                std::stringstream string_stream(line);

                // Timestamps are only ever skipped - pose loading does not use them.
                std::getline(string_stream, token, ',');
                if (has_two_timestamps)
                    std::getline(string_stream, token, ',');

                Point point{};

                std::getline(string_stream, token, ',');
                point.position.x = std::stof(token);

                std::getline(string_stream, token, ',');
                point.position.y = std::stof(token);

                std::getline(string_stream, token, ',');
                point.position.z = std::stof(token);

                TrajectoryPoseOrientationMat33 orientation{};

                if (is_quaternion)
                {
                    float q[4] = {};
                    for (float& value : q)
                    {
                        std::getline(string_stream, token, ',');
                        value = std::stof(token);
                    }

                    orientation.orientation = glm::mat3_cast(glm::quat(q[0], q[1], q[2], q[3]));
                }
                else
                {
                    float r[9] = {};
                    for (float& value : r)
                    {
                        std::getline(string_stream, token, ',');
                        value = std::stof(token);
                    }

                    orientation.orientation = glm::mat3(
                        r[0], r[1], r[2],
                        r[3], r[4], r[5],
                        r[6], r[7], r[8]);
                }

                trajectory_pose_positions.push_back(point);
                trajectory_pose_orientations.push_back(orientation);
            }
            catch (const std::exception& e)
            {
                spdlog::warn("Skipping malformed trajectory CSV line {}: {}", line_index, e.what());
            }

            line_index++;
        }

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

bool load_cave_laz(const std::filesystem::path& path, std::vector<PointIntensity>& points)
{
    points.clear();

    laszip_POINTER laszip_reader = nullptr;
    if (laszip_create(&laszip_reader))
    {
        spdlog::error("Failed to create LASzip reader");
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, path.string().c_str(), &is_compressed))
    {
        laszip_CHAR* error_msg = nullptr;
        laszip_get_error(laszip_reader, &error_msg);
        spdlog::error("Failed to open LAZ/LAS file: {} - {}", path.string(), error_msg ? error_msg : "Unknown error");
        laszip_destroy(laszip_reader);
        return false;
    }

    laszip_header_struct* header = nullptr;
    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        spdlog::error("Failed to get LASzip header pointer");
        laszip_close_reader(laszip_reader);
        laszip_destroy(laszip_reader);
        return false;
    }

    laszip_I64 num_points = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

    laszip_point_struct* point = nullptr;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        spdlog::error("Failed to get LASzip point pointer");
        laszip_close_reader(laszip_reader);
        laszip_destroy(laszip_reader);
        return false;
    }

    spdlog::info("Loading LAZ file '{}': {} points (compressed: {})", path.string(), num_points, is_compressed ? "yes" : "no");

    points.reserve(static_cast<size_t>(num_points));

    const double x_scale  = header->x_scale_factor;
    const double y_scale  = header->y_scale_factor;
    const double z_scale  = header->z_scale_factor;
    const double x_offset = header->x_offset;
    const double y_offset = header->y_offset;
    const double z_offset = header->z_offset;

    laszip_U16 min_intensity = std::numeric_limits<laszip_U16>::max();
    laszip_U16 max_intensity = std::numeric_limits<laszip_U16>::min();

    for (laszip_I64 i = 0; i < num_points; ++i)
    {
        if (laszip_read_point(laszip_reader))
        {
            spdlog::error("Failed to read point at index {}", i);
            break;
        }

        min_intensity = std::min(min_intensity, point->intensity);
        max_intensity = std::max(max_intensity, point->intensity);

        PointIntensity pt{};
        pt.position.x = static_cast<float>(point->X * x_scale + x_offset);
        pt.position.y = static_cast<float>(point->Y * y_scale + y_offset);
        pt.position.z = static_cast<float>(point->Z * z_scale + z_offset);
        pt.intensity  = static_cast<float>(point->intensity); // normalized to [0, 1] below once min/max are known across the whole file

        points.push_back(pt);
    }

    laszip_close_reader(laszip_reader);
    laszip_destroy(laszip_reader);

    const float intensity_range = static_cast<float>(max_intensity) - static_cast<float>(min_intensity);
    for (PointIntensity& pt : points)
        pt.intensity = (intensity_range > 0.0f) ? (pt.intensity - static_cast<float>(min_intensity)) / intensity_range : 1.0f;

    spdlog::info("Loaded {} points from LAZ file (intensity range [{}, {}])", points.size(), min_intensity, max_intensity);

    return !points.empty();
}
