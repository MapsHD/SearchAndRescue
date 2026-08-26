#pragma once

#include <Eigen/Eigen>
#include <glm/glm.hpp>

#include <vector>

bool write_affine3d_trajectory_to_txt(const std::vector<Eigen::Affine3d>& trajectory, const std::string& filepath);

std::string eigen_affine3d_to_string(const Eigen::Affine3d& T);

std::string glm_dmat4_to_string(const glm::dmat4& m);

Eigen::Affine3d glm_mat4_to_eigen_affine_3d(const glm::dmat4& matrix);