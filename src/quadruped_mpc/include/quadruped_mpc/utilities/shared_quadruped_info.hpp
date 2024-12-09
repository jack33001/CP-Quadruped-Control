#pragma once

#include <array>
#include <Eigen/Dense>
#include <mutex>
#include <memory>  // Include memory for unique_ptr

namespace quadruped_mpc {

struct HardwareInfo {
    double width{0.0};
    double height{0.0};
    double length{0.0};
};

struct QuadrupedState {
    // COM state
    Eigen::Vector3d p_c_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_c_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a_c_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond th_c_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d om_c_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d alpha_c_{Eigen::Vector3d::Zero()};

    // Foot positions
    Eigen::Vector3d p_1_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_2_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_3_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_4_{Eigen::Vector3d::Zero()};

    // Foot contact states
    bool contact_1_{false};
    bool contact_2_{false};
    bool contact_3_{false};
    bool contact_4_{false};
};

// TODO: define these values inside of a controller yaml instead of doing it here
struct BalanceControlInfo{
    // Controller gains
    double Kp_pos{0.0};
    double Kd_pos{0.0};
    double Kp_rot{0.0};
    double Kd_rot{0.0};

    // Desired controller behavior values
    double m{0.0};
    double g{0.0};
    Eigen::Matrix3d Ig{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d p_c_des{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_c_des{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond th_b_des{Eigen::Quaterniond::Identity()};  // Changed from Vector3d to Quaterniond
    Eigen::Vector3d om_b_des{Eigen::Vector3d::Zero()};

    // Other information
    Eigen::Matrix<double, 4, 4> F_star_prev{Eigen::Matrix<double, 4, 4>::Zero()};

    // Weights and constraints
    Eigen::Matrix<double, 6, 6> S{Eigen::Matrix<double, 6, 6>::Zero()};
    double alpha{0.0};
    double beta{0.0};
    Eigen::Matrix<double, 3, 4> C{Eigen::Matrix<double, 3, 4>::Zero()};
    Eigen::Matrix<double, 3, 3> d{Eigen::Matrix<double, 3, 3>::Zero()};
};

class SharedQuadrupedInfo {
public:
    static SharedQuadrupedInfo& getInstance();
    static void cleanup();  // Add cleanup method

    // Prevent copying
    SharedQuadrupedInfo(const SharedQuadrupedInfo&) = delete;
    void operator=(const SharedQuadrupedInfo&) = delete;

    HardwareInfo hardware_;
    QuadrupedState state_;
    BalanceControlInfo balance_;
    bool is_initialized_{false};  // Add initialization flag
    std::mutex mutex_;           // Add mutex for thread safety

private:
    SharedQuadrupedInfo() {  // Keep only one constructor
        // Initialize inertia matrix in constructor
        double width2 = std::pow(hardware_.width, 2);
        double height2 = std::pow(hardware_.height, 2);
        double length2 = std::pow(hardware_.length, 2);
        
        balance_.Ig << 
            (1.0/12.0) * balance_.m * (height2 + length2), 0.0, 0.0,
            0.0, (1.0/12.0) * balance_.m * (width2 + height2), 0.0,
            0.0, 0.0, (1.0/12.0) * balance_.m * (width2 + length2);
    }

    static std::unique_ptr<SharedQuadrupedInfo> instance_;
    static std::once_flag init_flag_;
    static std::mutex cleanup_mutex_;
};

// Note: There may be a segfault during system shutdown due to static destruction order.
// This is expected and does not affect runtime functionality.
extern SharedQuadrupedInfo& quadruped_info;

} // namespace quadruped_mpc
