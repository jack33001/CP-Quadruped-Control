#pragma once

#include <Eigen/Dense>
#include <array>
#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"  // Add this include
#include <acados_cpp/ocp.hpp>
#include <casadi/casadi.hpp>

namespace quadruped_mpc {

class BalanceController {
public:
    BalanceController() = default;
    ~BalanceController() = default;

    // Computes the 6x12 A matrix for the balance controller
    Eigen::Matrix<double, 6, 12> A();

    // Creates a 3x3 skew symmetric matrix from a 3D vector
    Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d& vec);

    // Computes the 6x1 desired wrench vector
    Eigen::Matrix<double, 6, 1> b_des();

    // Computes the 3x4 optimal force distribution
    Eigen::Matrix<double, 3, 4> F_star();

    // Implements the PD control law
    Eigen::Matrix<double, 6, 1> PD_law();

private:
    SharedQuadrupedInfo& quadruped_info_{SharedQuadrupedInfo::getInstance()};
    // Add private members as needed
};

} // namespace quadruped_mpc
