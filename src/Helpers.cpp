#include <cave-traversal-tool/Helpers.h>

#include <fstream>
#include <iomanip>
#include <sstream>

bool write_affine3d_trajectory_to_txt(const std::vector<Eigen::Affine3d>& trajectory, const std::string& filepath)
{
    if (std::ofstream file = std::ofstream(filepath))
    {

        file << std::fixed << std::setprecision(6);

        for (const auto& T : trajectory)
        {
            const Eigen::Vector3d t = T.translation();
            Eigen::Quaterniond    q(T.linear());
            q.normalize();

            file << t.x() << " "
                 << t.y() << " "
                 << t.z() << " "
                 << q.x() << " "
                 << q.y() << " "
                 << q.z() << " "
                 << q.w() << "\n";
        }

        return true;
    }

    return false;
}

std::string eigen_affine3d_to_string(const Eigen::Affine3d& T)
{
    std::ostringstream oss;
    oss << "\n"
        << T.matrix();
    return oss.str();
}

std::string glm_dmat4_to_string(const glm::dmat4& m)
{
    std::ostringstream oss;
    oss << "\n";
    for (int r = 0; r < 4; ++r)
    {
        oss << "[ ";
        for (int c = 0; c < 4; ++c)
            oss << std::setw(10) << m[c][r] << " ";
        oss << "]\n";
    }
    return oss.str();
}

Eigen::Affine3d glm_mat4_to_eigen_affine_3d(const glm::dmat4& matrix)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    for (uint32_t col = 0; col < 4; col++)
    {
        for (uint32_t row = 0; row < 4; row++)
        {
            result(row, col) = matrix[col][row];
        }
    }

    return Eigen::Affine3d(result);
}