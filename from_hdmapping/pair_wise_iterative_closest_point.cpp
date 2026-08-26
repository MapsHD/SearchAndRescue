#include "pair_wise_iterative_closest_point.h"
#include "hash_utils.h"
#include "point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h"
#include "structures.h"
#include "transformations.h"

#include <execution>
#include <iostream>
#include <mutex>

inline void relative_pose_tait_bryan_wc_case1_simplified_1(Eigen::Matrix<double, 6, 1>& relative_pose, double tx_1, double ty_1, double tz_1, double om_1, double fi_1, double ka_1, double tx_2, double ty_2, double tz_2, double om_2, double fi_2, double ka_2)
{
    double sin_om_1              = sin(om_1);
    double cos_om_1              = cos(om_1);
    double sin_fi_1              = sin(fi_1);
    double cos_fi_1              = cos(fi_1);
    double sin_ka_1              = sin(ka_1);
    double cos_ka_1              = cos(ka_1);
    double sin_om_2              = sin(om_2);
    double cos_om_2              = cos(om_2);
    double sin_fi_2              = sin(fi_2);
    double cos_fi_2              = cos(fi_2);
    double sin_ka_2              = sin(ka_2);
    double cos_ka_2              = cos(ka_2);
    double x0                    = cos_om_1 * sin_ka_1;
    double x1                    = cos_ka_1 * sin_om_1;
    double x2                    = sin_fi_1 * x1 + x0;
    double x3                    = sin_ka_1 * sin_om_1;
    double x4                    = cos_ka_1 * cos_om_1;
    double x5                    = sin_fi_1 * x4 - x3;
    double x6                    = cos_fi_1 * cos_ka_1;
    double x7                    = -sin_fi_1 * x3 + x4;
    double x8                    = sin_fi_1 * x0 + x1;
    double x9                    = cos_fi_1 * sin_ka_1;
    double x10                   = cos_fi_1 * cos_om_1;
    double x11                   = cos_fi_1 * sin_om_1;
    double x12                   = cos_fi_2 * sin_om_2;
    double x13                   = cos_fi_2 * cos_om_2;
    double x14                   = cos_ka_2 * sin_om_2;
    double x15                   = cos_om_2 * sin_ka_2;
    double x16                   = cos_ka_2 * cos_om_2;
    double x17                   = sin_ka_2 * sin_om_2;
    double x18                   = cos_fi_2 * x6;
    relative_pose.coeffRef(0, 0) = cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x6 - ty_1 * x2 + ty_2 * x2 + tz_1 * x5 - tz_2 * x5;
    relative_pose.coeffRef(1, 0) = tx_1 * x9 - tx_2 * x9 - ty_1 * x7 + ty_2 * x7 - tz_1 * x8 + tz_2 * x8;
    relative_pose.coeffRef(2, 0) = cos_fi_1 * cos_om_1 * tz_2 + cos_fi_1 * sin_om_1 * ty_1 - sin_fi_1 * tx_1 + sin_fi_1 * tx_2 - ty_2 * x11 - tz_1 * x10;
    relative_pose.coeffRef(3, 0) = atan2(sin_fi_2 * x9 + x12 * x7 - x13 * x8, sin_fi_1 * sin_fi_2 + x10 * x13 + x11 * x12);
    relative_pose.coeffRef(4, 0) = -asin(-sin_fi_2 * x6 + x12 * x2 + x13 * x5);
    relative_pose.coeffRef(5, 0) = atan2(sin_ka_2 * x18 - x2 * (-sin_fi_2 * x17 + x16) + x5 * (sin_fi_2 * x15 + x14), cos_ka_2 * x18 + x2 * (sin_fi_2 * x14 + x15) + x5 * (sin_fi_2 * x16 - x17));
}

inline void relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(Eigen::Matrix<double, 12, 12>& AtPA, const double& tx_1, const double& ty_1, const double& tz_1, const double& om_1, const double& fi_1, const double& ka_1, const double& tx_2, const double& ty_2, const double& tz_2, const double& om_2, const double& fi_2, const double& ka_2, const double& p_x, const double& p_y, const double& p_z, const double& p_om, const double& p_fi, const double& p_ka)
{
    double sin_om_1       = sin(om_1);
    double cos_om_1       = cos(om_1);
    double sin_fi_1       = sin(fi_1);
    double cos_fi_1       = cos(fi_1);
    double sin_ka_1       = sin(ka_1);
    double cos_ka_1       = cos(ka_1);
    double sin_om_2       = sin(om_2);
    double cos_om_2       = cos(om_2);
    double sin_fi_2       = sin(fi_2);
    double cos_fi_2       = cos(fi_2);
    double sin_ka_2       = sin(ka_2);
    double cos_ka_2       = cos(ka_2);
    double x0             = cos_fi_1 * cos_ka_1;
    double x1             = cos_om_1 * sin_ka_1;
    double x2             = cos_ka_1 * sin_om_1;
    double x3             = sin_fi_1 * x2 + x1;
    double x4             = cos_ka_1 * cos_om_1;
    double x5             = sin_fi_1 * x4 - sin_ka_1 * sin_om_1;
    double x6             = -x5;
    double x7             = cos_fi_1 * cos_om_1;
    double x8             = cos_fi_1 * sin_om_1;
    double x9             = -cos_fi_1 * cos_om_1 * tz_2 - cos_fi_1 * sin_om_1 * ty_1 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + ty_2 * x8 + tz_1 * x7;
    double x10            = -sin_fi_1 * sin_ka_1 * sin_om_1 + x4;
    double x11            = sin_fi_1 * x1 + x2;
    double x12            = cos_fi_1 * sin_ka_1;
    double x13            = cos_om_1 * sin_fi_1;
    double x14            = sin_fi_1 * sin_om_1;
    double x15            = cos_fi_2 * cos_om_2;
    double x16            = cos_fi_2 * sin_om_2;
    double x17            = sin_fi_1 * sin_fi_2 + x15 * x7 + x16 * x8;
    double x18            = sin_fi_2 * x12 + x10 * x16 - x11 * x15;
    double x19            = cos_fi_1 * x18 * (cos_om_1 * sin_om_2 - cos_om_2 * sin_om_1) + x17 * (cos_om_2 * x10 + sin_om_2 * x11);
    double x20            = pow(x17, 2);
    double x21            = 1.0 / (pow(x18, 2) + x20);
    double x22            = cos_fi_2 * x21;
    double x23            = cos_fi_2 * sin_fi_1;
    double x24            = x16 * x3;
    double x25            = sin_fi_2 * x0;
    double x26            = x15 * x5 + x24 - x25;
    double x27            = cos_fi_2 * x12;
    double x28            = cos_om_2 * sin_fi_2;
    double x29            = sin_fi_2 * sin_om_2;
    double x30            = cos_om_2 * x3 - sin_om_2 * x5;
    double x31            = pow(1 - pow(x15 * x6 - x24 + x25, 2), -1.0 / 2.0);
    double x32            = cos_fi_2 * x31;
    double x33            = cos_fi_2 * x0;
    double x34            = cos_ka_2 * sin_om_2;
    double x35            = cos_om_2 * sin_ka_2;
    double x36            = sin_fi_2 * x35 + x34;
    double x37            = cos_ka_2 * cos_om_2;
    double x38            = sin_ka_2 * sin_om_2;
    double x39            = -sin_fi_2 * x38 + x37;
    double x40            = sin_ka_2 * x33 - x3 * x39 + x36 * x5;
    double x41            = sin_fi_2 * x34 + x35;
    double x42            = sin_fi_2 * x37 - x38;
    double x43            = cos_ka_2 * x33 + x3 * x41 + x42 * x5;
    double x44            = 1.0 / (pow(x40, 2) + pow(x43, 2));
    double x45            = x40 * (-x3 * x42 + x41 * x5) + x43 * (x3 * x36 + x39 * x5);
    AtPA.coeffRef(0, 0)   = p_x * pow(x0, 2) + p_y * pow(x12, 2) + p_z * pow(sin_fi_1, 2);
    AtPA.coeffRef(0, 1)   = p_x * x0 * x3 - p_y * x10 * x12 - p_z * sin_fi_1 * x8;
    AtPA.coeffRef(0, 2)   = p_x * x0 * x6 - p_y * x11 * x12 + p_z * sin_fi_1 * x7;
    AtPA.coeffRef(0, 3)   = cos_fi_1 * p_z * sin_fi_1 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x0 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) - p_y * x12 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(0, 4)   = -cos_ka_1 * p_x * x0 * x9 - p_y * sin_ka_1 * x12 * x9 + p_z * sin_fi_1 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(0, 5)   = p_x * x0 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) - p_y * x12 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(0, 6)   = -p_x * pow(x0, 2) - p_y * pow(x12, 2) - p_z * pow(sin_fi_1, 2);
    AtPA.coeffRef(0, 7)   = -p_x * x0 * x3 + p_y * x10 * x12 + p_z * sin_fi_1 * x8;
    AtPA.coeffRef(0, 8)   = p_x * x0 * x5 + p_y * x11 * x12 - p_z * sin_fi_1 * x7;
    AtPA.coeffRef(0, 9)   = 0;
    AtPA.coeffRef(0, 10)  = 0;
    AtPA.coeffRef(0, 11)  = 0;
    AtPA.coeffRef(1, 0)   = p_x * x0 * x3 - p_y * x10 * x12 - p_z * sin_fi_1 * x8;
    AtPA.coeffRef(1, 1)   = p_x * pow(x3, 2) + p_y * pow(x10, 2) + p_z * pow(x8, 2);
    AtPA.coeffRef(1, 2)   = p_x * x3 * x6 + p_y * x10 * x11 - p_z * x7 * x8;
    AtPA.coeffRef(1, 3)   = -cos_fi_1 * p_z * x8 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x3 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_y * x10 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(1, 4)   = -cos_ka_1 * p_x * x3 * x9 + p_y * sin_ka_1 * x10 * x9 - p_z * x8 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(1, 5)   = p_x * x3 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * x10 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(1, 6)   = -p_x * x0 * x3 + p_y * x10 * x12 + p_z * sin_fi_1 * x8;
    AtPA.coeffRef(1, 7)   = -p_x * pow(x3, 2) - p_y * pow(x10, 2) - p_z * pow(x8, 2);
    AtPA.coeffRef(1, 8)   = p_x * x3 * x5 - p_y * x10 * x11 + p_z * x7 * x8;
    AtPA.coeffRef(1, 9)   = 0;
    AtPA.coeffRef(1, 10)  = 0;
    AtPA.coeffRef(1, 11)  = 0;
    AtPA.coeffRef(2, 0)   = p_x * x0 * x6 - p_y * x11 * x12 + p_z * sin_fi_1 * x7;
    AtPA.coeffRef(2, 1)   = p_x * x3 * x6 + p_y * x10 * x11 - p_z * x7 * x8;
    AtPA.coeffRef(2, 2)   = p_x * pow(x6, 2) + p_y * pow(x11, 2) + p_z * pow(x7, 2);
    AtPA.coeffRef(2, 3)   = cos_fi_1 * p_z * x7 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x6 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_y * x11 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(2, 4)   = -cos_ka_1 * p_x * x6 * x9 + p_y * sin_ka_1 * x11 * x9 + p_z * x7 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(2, 5)   = p_x * x6 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * x11 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(2, 6)   = -p_x * x0 * x6 + p_y * x11 * x12 - p_z * sin_fi_1 * x7;
    AtPA.coeffRef(2, 7)   = -p_x * x3 * x6 - p_y * x10 * x11 + p_z * x7 * x8;
    AtPA.coeffRef(2, 8)   = p_x * x5 * x6 - p_y * pow(x11, 2) - p_z * pow(x7, 2);
    AtPA.coeffRef(2, 9)   = 0;
    AtPA.coeffRef(2, 10)  = 0;
    AtPA.coeffRef(2, 11)  = 0;
    AtPA.coeffRef(3, 0)   = cos_fi_1 * p_z * sin_fi_1 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x0 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) - p_y * x12 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(3, 1)   = -cos_fi_1 * p_z * x8 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x3 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_y * x10 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(3, 2)   = cos_fi_1 * p_z * x7 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x6 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_y * x11 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(3, 3)   = pow(cos_fi_1, 2) * p_z * pow(-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2, 2) + p_fi * pow(x30, 2) * pow(x32, 2) + p_ka * pow(x44, 2) * pow(x45, 2) + p_om * pow(x19, 2) * pow(x22, 2) + p_x * pow(ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3, 2) + p_y * pow(-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10, 2);
    AtPA.coeffRef(3, 4)   = cos_fi_1 * p_z * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13) - cos_ka_1 * p_fi * x17 * x30 * x31 * x32 + cos_ka_1 * p_ka * pow(x44, 2) * x45 * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) - cos_ka_1 * p_x * x9 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_om * x19 * x21 * x22 * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23)) + p_y * sin_ka_1 * x9 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(3, 5)   = -p_fi * x18 * x30 * x31 * x32 + p_ka * pow(x44, 2) * x45 * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) + p_om * x17 * x19 * x21 * x22 * x26 + p_x * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10) * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(3, 6)   = -cos_fi_1 * p_z * sin_fi_1 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) - p_x * x0 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_y * x12 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(3, 7)   = cos_fi_1 * p_z * x8 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) - p_x * x3 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) - p_y * x10 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(3, 8)   = -cos_fi_1 * p_z * x7 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x5 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) - p_y * x11 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(3, 9)   = -p_fi * pow(x30, 2) * pow(x32, 2) - p_ka * pow(x44, 2) * pow(x45, 2) - p_om * pow(x19, 2) * pow(x22, 2);
    AtPA.coeffRef(3, 10)  = -p_fi * x30 * x31 * x32 * (-x28 * x5 - x29 * x3 - x33) + p_ka * x26 * pow(x44, 2) * x45 * (cos_ka_2 * x40 - sin_ka_2 * x43) + p_om * x19 * x21 * x22 * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(3, 11)  = -p_ka * x44 * x45;
    AtPA.coeffRef(4, 0)   = -cos_ka_1 * p_x * x0 * x9 - p_y * sin_ka_1 * x12 * x9 + p_z * sin_fi_1 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(4, 1)   = -cos_ka_1 * p_x * x3 * x9 + p_y * sin_ka_1 * x10 * x9 - p_z * x8 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(4, 2)   = -cos_ka_1 * p_x * x6 * x9 + p_y * sin_ka_1 * x11 * x9 + p_z * x7 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(4, 3)   = cos_fi_1 * p_z * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13) - cos_ka_1 * p_fi * x17 * x30 * x31 * x32 + cos_ka_1 * p_ka * pow(x44, 2) * x45 * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) - cos_ka_1 * p_x * x9 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_om * x19 * x21 * x22 * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23)) + p_y * sin_ka_1 * x9 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(4, 4)   = pow(cos_ka_1, 2) * p_fi * pow(x17, 2) * pow(x31, 2) + pow(cos_ka_1, 2) * p_ka * pow(x44, 2) * pow(x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8), 2) + pow(cos_ka_1, 2) * p_x * pow(x9, 2) + p_om * pow(x21, 2) * pow(sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23), 2) + p_y * pow(sin_ka_1, 2) * pow(x9, 2) + p_z * pow(cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13, 2);
    AtPA.coeffRef(4, 5)   = cos_ka_1 * p_fi * x17 * x18 * pow(x31, 2) + cos_ka_1 * p_ka * pow(x44, 2) * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) - cos_ka_1 * p_x * x9 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_om * x17 * pow(x21, 2) * x26 * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23)) + p_y * sin_ka_1 * x9 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(4, 6)   = cos_ka_1 * p_x * x0 * x9 + p_y * sin_ka_1 * x12 * x9 - p_z * sin_fi_1 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(4, 7)   = cos_ka_1 * p_x * x3 * x9 - p_y * sin_ka_1 * x10 * x9 + p_z * x8 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(4, 8)   = -cos_ka_1 * p_x * x5 * x9 - p_y * sin_ka_1 * x11 * x9 - p_z * x7 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(4, 9)   = cos_ka_1 * p_fi * x17 * x30 * x31 * x32 - cos_ka_1 * p_ka * pow(x44, 2) * x45 * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) - p_om * x19 * x21 * x22 * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23));
    AtPA.coeffRef(4, 10)  = cos_ka_1 * p_fi * x17 * pow(x31, 2) * (-x28 * x5 - x29 * x3 - x33) + cos_ka_1 * p_ka * x26 * pow(x44, 2) * (cos_ka_2 * x40 - sin_ka_2 * x43) * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) + p_om * pow(x21, 2) * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23)) * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(4, 11)  = -cos_ka_1 * p_ka * x44 * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8));
    AtPA.coeffRef(5, 0)   = p_x * x0 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) - p_y * x12 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 1)   = p_x * x3 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * x10 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 2)   = p_x * x6 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * x11 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 3)   = -p_fi * x18 * x30 * x31 * x32 + p_ka * pow(x44, 2) * x45 * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) + p_om * x17 * x19 * x21 * x22 * x26 + p_x * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10) * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 4)   = cos_ka_1 * p_fi * x17 * x18 * pow(x31, 2) + cos_ka_1 * p_ka * pow(x44, 2) * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) - cos_ka_1 * p_x * x9 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_om * x17 * pow(x21, 2) * x26 * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23)) + p_y * sin_ka_1 * x9 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 5)   = p_fi * pow(x18, 2) * pow(x31, 2) + p_ka * pow(x44, 2) * pow(-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36), 2) + p_om * pow(x17, 2) * pow(x21, 2) * pow(x26, 2) + p_x * pow(cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11, 2) + p_y * pow(cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5, 2);
    AtPA.coeffRef(5, 6)   = -p_x * x0 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * x12 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 7)   = -p_x * x3 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) - p_y * x10 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 8)   = p_x * x5 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) - p_y * x11 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(5, 9)   = p_fi * x18 * x30 * x31 * x32 - p_ka * pow(x44, 2) * x45 * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) - p_om * x17 * x19 * x21 * x22 * x26;
    AtPA.coeffRef(5, 10)  = p_fi * x18 * pow(x31, 2) * (-x28 * x5 - x29 * x3 - x33) + p_ka * x26 * pow(x44, 2) * (cos_ka_2 * x40 - sin_ka_2 * x43) * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) + p_om * x17 * pow(x21, 2) * x26 * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(5, 11)  = -p_ka * x44 * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36));
    AtPA.coeffRef(6, 0)   = -p_x * pow(x0, 2) - p_y * pow(x12, 2) - p_z * pow(sin_fi_1, 2);
    AtPA.coeffRef(6, 1)   = -p_x * x0 * x3 + p_y * x10 * x12 + p_z * sin_fi_1 * x8;
    AtPA.coeffRef(6, 2)   = -p_x * x0 * x6 + p_y * x11 * x12 - p_z * sin_fi_1 * x7;
    AtPA.coeffRef(6, 3)   = -cos_fi_1 * p_z * sin_fi_1 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) - p_x * x0 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) + p_y * x12 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(6, 4)   = cos_ka_1 * p_x * x0 * x9 + p_y * sin_ka_1 * x12 * x9 - p_z * sin_fi_1 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(6, 5)   = -p_x * x0 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) + p_y * x12 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(6, 6)   = p_x * pow(x0, 2) + p_y * pow(x12, 2) + p_z * pow(sin_fi_1, 2);
    AtPA.coeffRef(6, 7)   = p_x * x0 * x3 - p_y * x10 * x12 - p_z * sin_fi_1 * x8;
    AtPA.coeffRef(6, 8)   = -p_x * x0 * x5 - p_y * x11 * x12 + p_z * sin_fi_1 * x7;
    AtPA.coeffRef(6, 9)   = 0;
    AtPA.coeffRef(6, 10)  = 0;
    AtPA.coeffRef(6, 11)  = 0;
    AtPA.coeffRef(7, 0)   = -p_x * x0 * x3 + p_y * x10 * x12 + p_z * sin_fi_1 * x8;
    AtPA.coeffRef(7, 1)   = -p_x * pow(x3, 2) - p_y * pow(x10, 2) - p_z * pow(x8, 2);
    AtPA.coeffRef(7, 2)   = -p_x * x3 * x6 - p_y * x10 * x11 + p_z * x7 * x8;
    AtPA.coeffRef(7, 3)   = cos_fi_1 * p_z * x8 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) - p_x * x3 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) - p_y * x10 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(7, 4)   = cos_ka_1 * p_x * x3 * x9 - p_y * sin_ka_1 * x10 * x9 + p_z * x8 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(7, 5)   = -p_x * x3 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) - p_y * x10 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(7, 6)   = p_x * x0 * x3 - p_y * x10 * x12 - p_z * sin_fi_1 * x8;
    AtPA.coeffRef(7, 7)   = p_x * pow(x3, 2) + p_y * pow(x10, 2) + p_z * pow(x8, 2);
    AtPA.coeffRef(7, 8)   = -p_x * x3 * x5 + p_y * x10 * x11 - p_z * x7 * x8;
    AtPA.coeffRef(7, 9)   = 0;
    AtPA.coeffRef(7, 10)  = 0;
    AtPA.coeffRef(7, 11)  = 0;
    AtPA.coeffRef(8, 0)   = p_x * x0 * x5 + p_y * x11 * x12 - p_z * sin_fi_1 * x7;
    AtPA.coeffRef(8, 1)   = p_x * x3 * x5 - p_y * x10 * x11 + p_z * x7 * x8;
    AtPA.coeffRef(8, 2)   = p_x * x5 * x6 - p_y * pow(x11, 2) - p_z * pow(x7, 2);
    AtPA.coeffRef(8, 3)   = -cos_fi_1 * p_z * x7 * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) + p_x * x5 * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) - p_y * x11 * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10);
    AtPA.coeffRef(8, 4)   = -cos_ka_1 * p_x * x5 * x9 - p_y * sin_ka_1 * x11 * x9 - p_z * x7 * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13);
    AtPA.coeffRef(8, 5)   = p_x * x5 * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) - p_y * x11 * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5);
    AtPA.coeffRef(8, 6)   = -p_x * x0 * x5 - p_y * x11 * x12 + p_z * sin_fi_1 * x7;
    AtPA.coeffRef(8, 7)   = -p_x * x3 * x5 + p_y * x10 * x11 - p_z * x7 * x8;
    AtPA.coeffRef(8, 8)   = p_x * pow(x5, 2) + p_y * pow(x11, 2) + p_z * pow(x7, 2);
    AtPA.coeffRef(8, 9)   = 0;
    AtPA.coeffRef(8, 10)  = 0;
    AtPA.coeffRef(8, 11)  = 0;
    AtPA.coeffRef(9, 0)   = 0;
    AtPA.coeffRef(9, 1)   = 0;
    AtPA.coeffRef(9, 2)   = 0;
    AtPA.coeffRef(9, 3)   = -p_fi * pow(x30, 2) * pow(x32, 2) - p_ka * pow(x44, 2) * pow(x45, 2) - p_om * pow(x19, 2) * pow(x22, 2);
    AtPA.coeffRef(9, 4)   = cos_ka_1 * p_fi * x17 * x30 * x31 * x32 - cos_ka_1 * p_ka * pow(x44, 2) * x45 * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) - p_om * x19 * x21 * x22 * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23));
    AtPA.coeffRef(9, 5)   = p_fi * x18 * x30 * x31 * x32 - p_ka * pow(x44, 2) * x45 * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) - p_om * x17 * x19 * x21 * x22 * x26;
    AtPA.coeffRef(9, 6)   = 0;
    AtPA.coeffRef(9, 7)   = 0;
    AtPA.coeffRef(9, 8)   = 0;
    AtPA.coeffRef(9, 9)   = p_fi * pow(x30, 2) * pow(x32, 2) + p_ka * pow(x44, 2) * pow(x45, 2) + p_om * pow(x19, 2) * pow(x22, 2);
    AtPA.coeffRef(9, 10)  = p_fi * x30 * x31 * x32 * (-x28 * x5 - x29 * x3 - x33) - p_ka * x26 * pow(x44, 2) * x45 * (cos_ka_2 * x40 - sin_ka_2 * x43) - p_om * x19 * x21 * x22 * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(9, 11)  = p_ka * x44 * x45;
    AtPA.coeffRef(10, 0)  = 0;
    AtPA.coeffRef(10, 1)  = 0;
    AtPA.coeffRef(10, 2)  = 0;
    AtPA.coeffRef(10, 3)  = -p_fi * x30 * x31 * x32 * (-x28 * x5 - x29 * x3 - x33) + p_ka * x26 * pow(x44, 2) * x45 * (cos_ka_2 * x40 - sin_ka_2 * x43) + p_om * x19 * x21 * x22 * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(10, 4)  = cos_ka_1 * p_fi * x17 * pow(x31, 2) * (-x28 * x5 - x29 * x3 - x33) + cos_ka_1 * p_ka * x26 * pow(x44, 2) * (cos_ka_2 * x40 - sin_ka_2 * x43) * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) + p_om * pow(x21, 2) * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23)) * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(10, 5)  = p_fi * x18 * pow(x31, 2) * (-x28 * x5 - x29 * x3 - x33) + p_ka * x26 * pow(x44, 2) * (cos_ka_2 * x40 - sin_ka_2 * x43) * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) + p_om * x17 * pow(x21, 2) * x26 * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(10, 6)  = 0;
    AtPA.coeffRef(10, 7)  = 0;
    AtPA.coeffRef(10, 8)  = 0;
    AtPA.coeffRef(10, 9)  = p_fi * x30 * x31 * x32 * (-x28 * x5 - x29 * x3 - x33) - p_ka * x26 * pow(x44, 2) * x45 * (cos_ka_2 * x40 - sin_ka_2 * x43) - p_om * x19 * x21 * x22 * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPA.coeffRef(10, 10) = p_fi * pow(x31, 2) * pow(-x28 * x5 - x29 * x3 - x33, 2) + p_ka * pow(x26, 2) * pow(x44, 2) * pow(cos_ka_2 * x40 - sin_ka_2 * x43, 2) + p_om * pow(x21, 2) * pow(-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8), 2);
    AtPA.coeffRef(10, 11) = -p_ka * x26 * x44 * (cos_ka_2 * x40 - sin_ka_2 * x43);
    AtPA.coeffRef(11, 0)  = 0;
    AtPA.coeffRef(11, 1)  = 0;
    AtPA.coeffRef(11, 2)  = 0;
    AtPA.coeffRef(11, 3)  = -p_ka * x44 * x45;
    AtPA.coeffRef(11, 4)  = -cos_ka_1 * p_ka * x44 * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8));
    AtPA.coeffRef(11, 5)  = -p_ka * x44 * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36));
    AtPA.coeffRef(11, 6)  = 0;
    AtPA.coeffRef(11, 7)  = 0;
    AtPA.coeffRef(11, 8)  = 0;
    AtPA.coeffRef(11, 9)  = p_ka * x44 * x45;
    AtPA.coeffRef(11, 10) = -p_ka * x26 * x44 * (cos_ka_2 * x40 - sin_ka_2 * x43);
    AtPA.coeffRef(11, 11) = p_ka;
}

inline void relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(Eigen::Matrix<double, 12, 1>& AtPB, const double& tx_1, const double& ty_1, const double& tz_1, const double& om_1, const double& fi_1, const double& ka_1, const double& tx_2, const double& ty_2, const double& tz_2, const double& om_2, const double& fi_2, const double& ka_2, const double& tx_m, const double& ty_m, const double& tz_m, const double& om_m, const double& fi_m, const double& ka_m, const double& p_x, const double& p_y, const double& p_z, const double& p_om, const double& p_fi, const double& p_ka)
{
    double sin_om_1   = sin(om_1);
    double cos_om_1   = cos(om_1);
    double sin_fi_1   = sin(fi_1);
    double cos_fi_1   = cos(fi_1);
    double sin_ka_1   = sin(ka_1);
    double cos_ka_1   = cos(ka_1);
    double sin_om_2   = sin(om_2);
    double cos_om_2   = cos(om_2);
    double sin_fi_2   = sin(fi_2);
    double cos_fi_2   = cos(fi_2);
    double sin_ka_2   = sin(ka_2);
    double cos_ka_2   = cos(ka_2);
    double a0         = cos_om_1 * sin_ka_1;
    double a1         = cos_ka_1 * sin_om_1;
    double a2         = a0 + a1 * sin_fi_1;
    double a3         = sin_ka_1 * sin_om_1;
    double a4         = cos_ka_1 * cos_om_1;
    double a5         = -a3 + a4 * sin_fi_1;
    double a6         = cos_fi_1 * cos_ka_1;
    double a7         = -a3 * sin_fi_1 + a4;
    double a8         = a0 * sin_fi_1 + a1;
    double a9         = cos_fi_1 * sin_ka_1;
    double a10        = cos_fi_1 * cos_om_1;
    double a11        = cos_fi_1 * sin_om_1;
    double a12        = cos_fi_2 * sin_om_2;
    double a13        = cos_fi_2 * cos_om_2;
    double a14        = cos_ka_2 * sin_om_2;
    double a15        = cos_om_2 * sin_ka_2;
    double a16        = cos_ka_2 * cos_om_2;
    double a17        = sin_ka_2 * sin_om_2;
    double a18        = a6 * cos_fi_2;
    double x0         = cos_fi_1 * cos_ka_1;
    double x1         = cos_om_1 * sin_ka_1;
    double x2         = cos_ka_1 * sin_om_1;
    double x3         = sin_fi_1 * x2 + x1;
    double x4         = cos_ka_1 * cos_om_1;
    double x5         = sin_fi_1 * x4 - sin_ka_1 * sin_om_1;
    double x6         = -x5;
    double x7         = cos_fi_1 * cos_om_1;
    double x8         = cos_fi_1 * sin_om_1;
    double x9         = -cos_fi_1 * cos_om_1 * tz_2 - cos_fi_1 * sin_om_1 * ty_1 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + ty_2 * x8 + tz_1 * x7;
    double x10        = -sin_fi_1 * sin_ka_1 * sin_om_1 + x4;
    double x11        = sin_fi_1 * x1 + x2;
    double x12        = cos_fi_1 * sin_ka_1;
    double x13        = cos_om_1 * sin_fi_1;
    double x14        = sin_fi_1 * sin_om_1;
    double x15        = cos_fi_2 * cos_om_2;
    double x16        = cos_fi_2 * sin_om_2;
    double x17        = sin_fi_1 * sin_fi_2 + x15 * x7 + x16 * x8;
    double x18        = sin_fi_2 * x12 + x10 * x16 - x11 * x15;
    double x19        = cos_fi_1 * x18 * (cos_om_1 * sin_om_2 - cos_om_2 * sin_om_1) + x17 * (cos_om_2 * x10 + sin_om_2 * x11);
    double x20        = pow(x17, 2);
    double x21        = 1.0 / (pow(x18, 2) + x20);
    double x22        = cos_fi_2 * x21;
    double x23        = cos_fi_2 * sin_fi_1;
    double x24        = x16 * x3;
    double x25        = sin_fi_2 * x0;
    double x26        = x15 * x5 + x24 - x25;
    double x27        = cos_fi_2 * x12;
    double x28        = cos_om_2 * sin_fi_2;
    double x29        = sin_fi_2 * sin_om_2;
    double x30        = cos_om_2 * x3 - sin_om_2 * x5;
    double x31        = pow(1 - pow(x15 * x6 - x24 + x25, 2), -1.0 / 2.0);
    double x32        = cos_fi_2 * x31;
    double x33        = cos_fi_2 * x0;
    double x34        = cos_ka_2 * sin_om_2;
    double x35        = cos_om_2 * sin_ka_2;
    double x36        = sin_fi_2 * x35 + x34;
    double x37        = cos_ka_2 * cos_om_2;
    double x38        = sin_ka_2 * sin_om_2;
    double x39        = -sin_fi_2 * x38 + x37;
    double x40        = sin_ka_2 * x33 - x3 * x39 + x36 * x5;
    double x41        = sin_fi_2 * x34 + x35;
    double x42        = sin_fi_2 * x37 - x38;
    double x43        = cos_ka_2 * x33 + x3 * x41 + x42 * x5;
    double x44        = 1.0 / (pow(x40, 2) + pow(x43, 2));
    double x45        = x40 * (-x3 * x42 + x41 * x5) + x43 * (x3 * x36 + x39 * x5);
    AtPB.coeffRef(0)  = p_x * x0 * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) - p_y * x12 * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m) + p_z * sin_fi_1 * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m);
    AtPB.coeffRef(1)  = p_x * x3 * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) + p_y * x10 * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m) - p_z * x8 * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m);
    AtPB.coeffRef(2)  = p_x * x6 * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) + p_y * x11 * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m) + p_z * x7 * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m);
    AtPB.coeffRef(3)  = cos_fi_1 * p_z * (-cos_om_1 * ty_1 + cos_om_1 * ty_2 - sin_om_1 * tz_1 + sin_om_1 * tz_2) * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m) - p_fi * x30 * x32 * (fi_m + asin(a12 * a2 + a13 * a5 - a6 * sin_fi_2)) + p_ka * x44 * x45 * (ka_m - atan2(a18 * sin_ka_2 - a2 * (a16 - a17 * sin_fi_2) + a5 * (a14 + a15 * sin_fi_2), a18 * cos_ka_2 + a2 * (a14 * sin_fi_2 + a15) + a5 * (a16 * sin_fi_2 - a17))) + p_om * x19 * x22 * (om_m - atan2(a12 * a7 - a13 * a8 + a9 * sin_fi_2, a10 * a13 + a11 * a12 + sin_fi_1 * sin_fi_2)) + p_x * (ty_1 * x5 - ty_2 * x5 + tz_1 * x3 - tz_2 * x3) * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) + p_y * (-ty_1 * x11 + ty_2 * x11 + tz_1 * x10 - tz_2 * x10) * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m);
    AtPB.coeffRef(4)  = cos_ka_1 * p_fi * x17 * x31 * (fi_m + asin(a12 * a2 + a13 * a5 - a6 * sin_fi_2)) + cos_ka_1 * p_ka * x44 * (ka_m - atan2(a18 * sin_ka_2 - a2 * (a16 - a17 * sin_fi_2) + a5 * (a14 + a15 * sin_fi_2), a18 * cos_ka_2 + a2 * (a14 * sin_fi_2 + a15) + a5 * (a16 * sin_fi_2 - a17))) * (x40 * (-cos_ka_2 * x23 + x41 * x8 + x42 * x7) + x43 * (sin_ka_2 * x23 - x36 * x7 + x39 * x8)) - cos_ka_1 * p_x * x9 * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) + p_om * x21 * (om_m - atan2(a12 * a7 - a13 * a8 + a9 * sin_fi_2, a10 * a13 + a11 * a12 + sin_fi_1 * sin_fi_2)) * (sin_ka_1 * x20 - x18 * (-cos_fi_1 * sin_fi_2 + cos_om_1 * cos_om_2 * x23 + sin_om_1 * sin_om_2 * x23)) + p_y * sin_ka_1 * x9 * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m) + p_z * (cos_fi_1 * tx_1 - cos_fi_1 * tx_2 + ty_1 * x14 - ty_2 * x14 - tz_1 * x13 + tz_2 * x13) * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m);
    AtPB.coeffRef(5)  = p_fi * x18 * x31 * (fi_m + asin(a12 * a2 + a13 * a5 - a6 * sin_fi_2)) + p_ka * x44 * (ka_m - atan2(a18 * sin_ka_2 - a2 * (a16 - a17 * sin_fi_2) + a5 * (a14 + a15 * sin_fi_2), a18 * cos_ka_2 + a2 * (a14 * sin_fi_2 + a15) + a5 * (a16 * sin_fi_2 - a17))) * (-x40 * (cos_ka_2 * x27 - x10 * x41 + x11 * x42) + x43 * (sin_ka_2 * x27 + x10 * x39 + x11 * x36)) + p_om * x17 * x21 * x26 * (om_m - atan2(a12 * a7 - a13 * a8 + a9 * sin_fi_2, a10 * a13 + a11 * a12 + sin_fi_1 * sin_fi_2)) + p_x * (cos_fi_1 * sin_ka_1 * tx_2 - tx_1 * x12 + ty_1 * x10 - ty_2 * x10 + tz_1 * x11 - tz_2 * x11) * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) + p_y * (cos_fi_1 * cos_ka_1 * tx_2 - tx_1 * x0 - ty_1 * x3 + ty_2 * x3 + tz_1 * x5 - tz_2 * x5) * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m);
    AtPB.coeffRef(6)  = -p_x * x0 * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) + p_y * x12 * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m) - p_z * sin_fi_1 * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m);
    AtPB.coeffRef(7)  = -p_x * x3 * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) - p_y * x10 * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m) + p_z * x8 * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m);
    AtPB.coeffRef(8)  = p_x * x5 * (a2 * ty_1 - a2 * ty_2 - a5 * tz_1 + a5 * tz_2 + a6 * tx_1 - a6 * tx_2 + tx_m) - p_y * x11 * (a7 * ty_1 - a7 * ty_2 + a8 * tz_1 - a8 * tz_2 - a9 * tx_1 + a9 * tx_2 + ty_m) - p_z * x7 * (a10 * tz_1 - a10 * tz_2 - a11 * ty_1 + a11 * ty_2 + sin_fi_1 * tx_1 - sin_fi_1 * tx_2 + tz_m);
    AtPB.coeffRef(9)  = p_fi * x30 * x32 * (fi_m + asin(a12 * a2 + a13 * a5 - a6 * sin_fi_2)) - p_ka * x44 * x45 * (ka_m - atan2(a18 * sin_ka_2 - a2 * (a16 - a17 * sin_fi_2) + a5 * (a14 + a15 * sin_fi_2), a18 * cos_ka_2 + a2 * (a14 * sin_fi_2 + a15) + a5 * (a16 * sin_fi_2 - a17))) - p_om * x19 * x22 * (om_m - atan2(a12 * a7 - a13 * a8 + a9 * sin_fi_2, a10 * a13 + a11 * a12 + sin_fi_1 * sin_fi_2));
    AtPB.coeffRef(10) = p_fi * x31 * (fi_m + asin(a12 * a2 + a13 * a5 - a6 * sin_fi_2)) * (-x28 * x5 - x29 * x3 - x33) + p_ka * x26 * x44 * (ka_m - atan2(a18 * sin_ka_2 - a2 * (a16 - a17 * sin_fi_2) + a5 * (a14 + a15 * sin_fi_2), a18 * cos_ka_2 + a2 * (a14 * sin_fi_2 + a15) + a5 * (a16 * sin_fi_2 - a17))) * (cos_ka_2 * x40 - sin_ka_2 * x43) + p_om * x21 * (om_m - atan2(a12 * a7 - a13 * a8 + a9 * sin_fi_2, a10 * a13 + a11 * a12 + sin_fi_1 * sin_fi_2)) * (-x17 * (-x10 * x29 + x11 * x28 + x27) - x18 * (-x23 + x28 * x7 + x29 * x8));
    AtPB.coeffRef(11) = -p_ka * (ka_m - atan2(a18 * sin_ka_2 - a2 * (a16 - a17 * sin_fi_2) + a5 * (a14 + a15 * sin_fi_2), a18 * cos_ka_2 + a2 * (a14 * sin_fi_2 + a15) + a5 * (a16 * sin_fi_2 - a17)));
}

bool PairWiseICP::compute(const std::vector<Eigen::Vector3d>& source, const std::vector<Eigen::Vector3d>& target, const std::vector<Eigen::Vector3d>& target_normals, double search_radious, int number_of_iterations, Eigen::Affine3d& m_pose_result, bool is_repulsion, const Eigen::Affine3d& m_previous_pose)
{
    bool multithread = true;

    for (int iter = 0; iter < number_of_iterations; iter++)
    {
        std::vector<Eigen::Affine3d> intermediate_trajectory;
        intermediate_trajectory.push_back(m_previous_pose);
        intermediate_trajectory.push_back(m_pose_result);

        std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model = intermediate_trajectory;

        Eigen::MatrixXd AtPA(12, 12);
        AtPA.setZero();
        Eigen::MatrixXd AtPB(12, 1);
        AtPB.setZero();

        std::vector<std::pair<int, int>> odo_edges;
        odo_edges.reserve(intermediate_trajectory.size());
        for (size_t i = 1; i < intermediate_trajectory.size(); i++)
        {
            odo_edges.emplace_back(i - 1, i);
        }

        std::vector<TaitBryanPose> poses;
        poses.reserve(intermediate_trajectory.size());

        std::vector<TaitBryanPose> poses_desired;
        poses_desired.reserve(intermediate_trajectory_motion_model.size());

        for (size_t i = 0; i < intermediate_trajectory.size(); i++)
            poses.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]));

        for (size_t i = 0; i < intermediate_trajectory_motion_model.size(); i++)
            poses_desired.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory_motion_model[i]));

        for (size_t i = 0; i < odo_edges.size(); i++)
        {
            Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
            relative_pose_tait_bryan_wc_case1_simplified_1(relative_pose_measurement_odo, poses_desired[odo_edges[i].first].px, poses_desired[odo_edges[i].first].py, poses_desired[odo_edges[i].first].pz, poses_desired[odo_edges[i].first].om, poses_desired[odo_edges[i].first].fi, poses_desired[odo_edges[i].first].ka, poses_desired[odo_edges[i].second].px, poses_desired[odo_edges[i].second].py, poses_desired[odo_edges[i].second].pz, poses_desired[odo_edges[i].second].om, poses_desired[odo_edges[i].second].fi, poses_desired[odo_edges[i].second].ka);

            Eigen::Matrix<double, 12, 12> AtPAodo;
            relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(
                AtPAodo,
                poses[odo_edges[i].first].px,
                poses[odo_edges[i].first].py,
                poses[odo_edges[i].first].pz,
                poses[odo_edges[i].first].om,
                poses[odo_edges[i].first].fi,
                poses[odo_edges[i].first].ka,
                poses[odo_edges[i].second].px,
                poses[odo_edges[i].second].py,
                poses[odo_edges[i].second].pz,
                poses[odo_edges[i].second].om,
                poses[odo_edges[i].second].fi,
                poses[odo_edges[i].second].ka,
                10000,
                10000,
                10000,
                100,
                100,
                100);

            Eigen::Matrix<double, 12, 1> AtPBodo;
            relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(
                AtPBodo,
                poses[odo_edges[i].first].px,
                poses[odo_edges[i].first].py,
                poses[odo_edges[i].first].pz,
                poses[odo_edges[i].first].om,
                poses[odo_edges[i].first].fi,
                poses[odo_edges[i].first].ka,
                poses[odo_edges[i].second].px,
                poses[odo_edges[i].second].py,
                poses[odo_edges[i].second].pz,
                poses[odo_edges[i].second].om,
                poses[odo_edges[i].second].fi,
                poses[odo_edges[i].second].ka,
                relative_pose_measurement_odo(0, 0),
                relative_pose_measurement_odo(1, 0),
                relative_pose_measurement_odo(2, 0),
                relative_pose_measurement_odo(3, 0),
                relative_pose_measurement_odo(4, 0),
                relative_pose_measurement_odo(5, 0),
                10000,
                10000,
                10000,
                100,
                100,
                100);

            int ic_1 = odo_edges[i].first * 6;
            int ic_2 = odo_edges[i].second * 6;

            for (int row = 0; row < 6; row++)
            {
                for (int col = 0; col < 6; col++)
                {
                    AtPA(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
                    AtPA(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
                    AtPA(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
                    AtPA(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
                }
            }

            for (int row = 0; row < 6; row++)
            {
                AtPB(ic_1 + row, 0) -= AtPBodo(row, 0);
                AtPB(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
            }
        }

        std::vector<std::pair<unsigned long long int, unsigned int>> indexes;
        indexes.reserve(target.size());

        for (int i = 0; i < target.size(); i++)
        {
            unsigned long long int index = get_rgd_index(target[i], {search_radious, search_radious, search_radious});
            indexes.emplace_back(index, i);
        }

        std::sort(indexes.begin(), indexes.end(),
                  [](const std::pair<unsigned long long int, unsigned int>& a, const std::pair<unsigned long long int, unsigned int>& b)
                  { return a.first < b.first; });

        std::unordered_map<unsigned long long int, std::pair<unsigned int, unsigned int>> buckets;

        for (unsigned int i = 0; i < indexes.size(); i++)
        {
            unsigned long long int index_of_bucket = indexes[i].first;
            if (buckets.contains(index_of_bucket))
            {
                buckets[index_of_bucket].second = i;
            }
            else
            {
                buckets[index_of_bucket].first  = i;
                buckets[index_of_bucket].second = i;
            }
        }

        std::mutex mutex;
        int        counter_nn = 0;

        const auto hessian_fun = [&](const Eigen::Vector3d& source_i)
        {
            if (source_i.norm() < 0.1)
            {
                return;
            }

            Eigen::Vector3d source_point_global = m_pose_result * source_i;

            double          min_dist = 1000000000.0;
            Eigen::Vector3d target_i;
            Eigen::Vector3d target_i_nn;
            bool            nn_found = false;
            for (double x = -search_radious; x <= search_radious; x += search_radious)
            {
                for (double y = -search_radious; y <= search_radious; y += search_radious)
                {
                    for (double z = -search_radious; z <= search_radious; z += search_radious)
                    {
                        Eigen::Vector3d        source_point_global_ext = source_point_global + Eigen::Vector3d(x, y, z);
                        unsigned long long int index_of_bucket         = get_rgd_index(source_point_global_ext, {search_radious, search_radious, search_radious});

                        if (buckets.contains(index_of_bucket))
                        {
                            for (int index = buckets[index_of_bucket].first; index < buckets[index_of_bucket].second; index++)
                            {
                                int index_element_target = indexes[index].second;
                                target_i                 = target[index_element_target];
                                double d                 = (source_point_global - target_i).norm();

                                if (d < search_radious)
                                {
                                    if (d < min_dist)
                                    {
                                        min_dist    = d;
                                        nn_found    = true;
                                        target_i_nn = target_i;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (nn_found)
            {
                const Eigen::Vector3d& p_s    = source_i;
                const TaitBryanPose    pose_s = pose_tait_bryan_from_affine_matrix(m_pose_result);

                Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA_;
                point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                    AtPA_,
                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                    p_s.x(), p_s.y(), p_s.z(),
                    1, 0, 0, 0, 1, 0, 0, 0, 1);

                Eigen::Matrix<double, 6, 1> AtPB_;
                point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                    AtPB_,
                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                    p_s.x(), p_s.y(), p_s.z(),
                    1, 0, 0, 0, 1, 0, 0, 0, 1,
                    target_i_nn.x(), target_i_nn.y(), target_i_nn.z());

                std::unique_lock lck(mutex);

                int ic_1 = 6;
                int ic_2 = 6;

                for (int row = 0; row < 6; row++)
                {
                    for (int col = 0; col < 6; col++)
                    {
                        AtPA(ic_1 + row, ic_1 + col) += AtPA_(row, col);
                    }
                }

                if (!is_repulsion)
                {
                    for (int row = 0; row < 6; row++)
                    {
                        AtPB(ic_2 + row, 0) -= AtPA_(row + 6, 0);
                    }
                }
                else
                {
                    for (int row = 0; row < 6; row++)
                    {
                        AtPB(ic_2 + row, 0) += AtPA_(row + 6, 0);
                    }
                }
                counter_nn++;
            }
        };

        if (multithread)
        {
            std::for_each(std::execution::par_unseq, std::begin(source), std::end(source), hessian_fun);
        }
        else
        {
            std::for_each(std::begin(source), std::end(source), hessian_fun);
        }

        if (AtPB(0, 0) == 0.0 && AtPB(1, 0) == 0.0 && AtPB(2, 0) == 0.0 && AtPB(3, 0) == 0.0 && AtPB(4, 0) == 0.0 && AtPB(5, 0) == 0.0)
        {
            return false;
        }

        if (counter_nn < 100)
        {
            return false;
        }

        for (int row = 0; row < 6; row++)
        {
            AtPA(row, row) += 10000;
        }

        Eigen::SparseMatrix<double> AtPAc(12, 12);
        Eigen::SparseMatrix<double> AtPBc(12, 1);

        AtPAc = AtPA.sparseView();
        AtPBc = AtPB.sparseView();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPAc);
        Eigen::SparseMatrix<double>                            x = solver.solve(AtPBc);
        std::vector<double>                                    h_x;
        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == 12)
        {
            int           counter = 6;
            TaitBryanPose pose    = pose_tait_bryan_from_affine_matrix(m_pose_result);

#if 1
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
#else
            counter++;
            counter++;
            counter++;
#endif
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];

            m_pose_result = affine_matrix_from_pose_tait_bryan(pose);
            // return true;
        }
        else
        {
            return false;
        }
    }

    return true;
}
