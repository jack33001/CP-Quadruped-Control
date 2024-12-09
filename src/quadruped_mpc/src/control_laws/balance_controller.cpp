#include "quadruped_mpc/control_laws/balance_controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <mutex>

// This controller is based off of the balance controller from the paper:
// "MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot"

namespace quadruped_mpc {

Eigen::Matrix<double, 6, 12> BalanceController::A() {  
    Eigen::Matrix<double, 6, 12> A_matrix;
    Eigen::Vector3d p_com;
    Eigen::Vector3d p1, p2, p3, p4;

    // Lock the mutex while accessing shared data
    {
        std::lock_guard<std::mutex> lock(quadruped_info_.mutex_);

        // Access foot positions
        p1 = quadruped_info_.state_.p_1_;
        p2 = quadruped_info_.state_.p_2_;
        p3 = quadruped_info_.state_.p_3_;
        p4 = quadruped_info_.state_.p_4_;

        // Access COM state
        p_com = quadruped_info_.state_.p_c_;
    }

    // Fill the matrix
    A_matrix.block<3,3>(0,0).setIdentity();
    A_matrix.block<3,3>(0,3).setIdentity();
    A_matrix.block<3,3>(0,6).setIdentity();
    A_matrix.block<3,3>(0,9).setIdentity();
    A_matrix.block<3,3>(3,0) = skew_symmetric_matrix(p1-p_com);
    A_matrix.block<3,3>(3,3) = skew_symmetric_matrix(p2-p_com);
    A_matrix.block<3,3>(3,6) = skew_symmetric_matrix(p3-p_com);
    A_matrix.block<3,3>(3,9) = skew_symmetric_matrix(p4-p_com);

    return A_matrix;
}

Eigen::Matrix3d BalanceController::skew_symmetric_matrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew;
    
    // Calculate the skew-symmetric matrix to represent the cross product with the input vector
    skew <<     0, -vec[2],  vec[1],
           vec[2],      0, -vec[0],
          -vec[1],  vec[0],      0;
    return skew;
}

Eigen::Matrix<double, 6, 1> BalanceController::b_des() {
    Eigen::Matrix<double, 6, 1> b;
    Eigen::Matrix<double, 6, 1> effort;
    double m, g, Ig;

    // Lock the mutex while accessing shared data
    {
        std::lock_guard<std::mutex> lock(quadruped_info_.mutex_);
        m = quadruped_info_.balance_.m;
        g = quadruped_info_.balance_.g;
        Ig = quadruped_info_.balance_.Ig;
    }

    // Calculate the controller effort
    effort = PD_law();
    
    // Calculate the desired linear system behavior
    b.block<3,1>(0,0) = ((effort.block<3,1>(0,0).array() + g) * m).matrix();
    // Calculate the desired rotational system behavior
    b.block<3,1>(3,0) = (effort.block<3,1>(3,0).array() * Ig).matrix();
    
    return b;
}

Eigen::Matrix<double, 3, 4> BalanceController::F_star(Eigen::Matrix<double, 3, 4> F) {
    // System behaviors
    Eigen::Matrix<double, 3, 4> F_star;              // The desired foot forces
    Eigen::Matrix<double, 3,4> F_star_prev;     // The previous desired foot forces
    Eigen::Matrix<double, 6, 1> b_des;          // The desired system behavior
    Eigen::Matrix<double, 6, 12> A_matrix;      // The A matrix
    // Weights
    Eigen::Matrix<double, 6, 6> S;              // The relative priority matrix (translational vs rotational)
    double alpha, beta;                         // Force normalization and solution filtering weights
    // Constraints
    Eigen::Matrix<double, 3, 4> C = Eigen::Matrix<double, 3, 4>::Ones();  // Constraint weights
    Eigen::Matrix<double, 3, 3> d = Eigen::Matrix<double, 3, 3>::Ones();  // Feasibility constraint
    
    // Lock the mutex while accessing shared data
    {
        std::lock_guard<std::mutex> lock(quadruped_info_.mutex_);
        F_star_prev = quadruped_info_.balance_.F_star_prev;
        alpha = quadruped_info_.balance_.alpha;
        beta = quadruped_info_.balance_.beta;
        S = quadruped_info_.balance_.S;
        Eigen::Matrix<double, 1, 3> C_row_support = quadruped_info_.balance_.C_row_support;
        Eigen::Matrix<double, 1, 3> C_row_swing = quadruped_info_.balance_.C_row_swing;
        Eigen::Matrix<double, 1, 3> d_row_support = quadruped_info_.balance_.d_row_support;
        Eigen::Matrix<double, 1, 3> d_row_swing = quadruped_info_.balance_.d_row_swing;
    }

    // Calculate the variables within the control law
    b_des = b_des();
    A_matrix = A();

    // Pack the constrain matrices
    // TODO: Implement building constraint matrices

    // Calculate the desired foot forces
    F_star = (A_matrix * F - b_des).transpose() * S * (A_matrix * F - b_des) + alpha * (F.norm() * F.norm()) + beta * (F - F_star_prev).norm()

    return F_star;
}

Eigen::Matrix<double, 6, 1> BalanceController::PD_law() {
    Eigen::Matrix<double, 6, 1> pd;
    double Kp_pos, Kd_pos, Kp_rot, Kd_rot;
    Eigen::Vector3d p_c_des, v_c_des, om_b_des, p_c, v_c, om_b;
    Eigen::Quaterniond th_b, th_b_des;

    // Get information from the storage struct
    {
        std::lock_guard<std::mutex> lock(quadruped_info_.mutex_);

        // Pull out gains
        Kp_pos = quadruped_info_.balance_.Kp_pos
        Kd_pos = quadruped_info_.balance_.Kd_pos
        Kp_rot = quadruped_info_.balance_.Kp_rot
        Kd_rot = quadruped_info_.balance_.Kd_rot

        // Pull out desired valuess
        p_c_des = quadruped_info_.balance_.p_c_des
        v_c_des = quadruped_info_.balance_.v_c_des
        th_b_des = quadruped_info_.balance_.th_b_des
        om_b_des = quadruped_info_.balance_.om_b_des

        // Pull out actual values
        p_c_ = quadruped_info_.state_.p_c_
        v_c_ = quadruped_info_.state_.v_c_
        th_b = quadruped_info_.state_.th_b_
        om_b = quadruped_info_.state_.angular_velocity
    }
    
    // Calculate the linear effort
    pd.block<3,1>(0,0) = Kp_pos * (p_c_des - p_c) + Kd_pos * (v_c_des - v_c)
    // Calculate the rotational effort
    pd.block<3,1>(3,0) = Kp_rot * ((th_b_des.toRotationMatrix() * th_b.toRotationMatrix().transpose()).array().log()).matrix() + Kd_rot * (om_b_des - om_b)

    return pd;
}

} // namespace quadruped_mpc
