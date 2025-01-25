#pragma once

#include <array>
#include <Eigen/Dense>
#include <mutex>
#include <memory>  // Include memory for unique_ptr
#include <pinocchio/fwd.hpp>  // Add Pinocchio headers
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace quadruped_mpc {

struct HardwareInfo {
    double width{0.0};
    double height{0.0};
    double length{0.0};
};

struct QuadrupedState {
    // Replace Pinocchio data with cartesian positions and joint states
    Eigen::Vector3d p1{Eigen::Vector3d::Zero()};  // foot 1 position
    Eigen::Vector3d p2{Eigen::Vector3d::Zero()};  // foot 2 position
    Eigen::Vector3d p3{Eigen::Vector3d::Zero()};  // foot 3 position
    Eigen::Vector3d p4{Eigen::Vector3d::Zero()};  // foot 4 position
    Eigen::Vector3d pc{Eigen::Vector3d::Zero()};  // body position

    std::array<double, 8> joint_pos{};  // knee and hip positions
    std::array<double, 8> joint_vel{};  // knee and hip velocities
    std::array<double, 8> joint_eff{};  // knee and hip efforts

    // Add IMU state - replace euler angles with quaternion
    Eigen::Vector4d orientation_quat{1.0, 0.0, 0.0, 0.0};  // quaternion in w,x,y,z order
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()}; // angular velocity
    
    // Add velocity state
    Eigen::Vector3d vc{Eigen::Vector3d::Zero()};  // center of mass velocity

    // Add Jacobian storage (one 3x2 Jacobian per leg)
    // Each Jacobian maps joint velocities to foot velocities for that leg
    Eigen::Matrix<double, 3, 2> J1{Eigen::Matrix<double, 3, 2>::Zero()};  // FL Jacobian
    Eigen::Matrix<double, 3, 2> J2{Eigen::Matrix<double, 3, 2>::Zero()};  // FR Jacobian
    Eigen::Matrix<double, 3, 2> J3{Eigen::Matrix<double, 3, 2>::Zero()};  // RL Jacobian
    Eigen::Matrix<double, 3, 2> J4{Eigen::Matrix<double, 3, 2>::Zero()};  // RR Jacobian

    // Default constructor
    QuadrupedState() = default;

    // Contact states only
    bool contact_1_{false};
    bool contact_2_{false};
    bool contact_3_{false};
    bool contact_4_{false};
};

// TODO: define these values inside of a controller yaml instead of doing it here
struct BalanceControlInfo {
    double mass;
    Eigen::Matrix3d inertia;
    std::array<Eigen::Vector3d, 4> feet_positions;

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

struct RobotParameters {
    double mass{0.0};
    Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
    std::vector<Eigen::Vector3d> leg_positions{
        Eigen::Vector3d::Zero(),  // FL
        Eigen::Vector3d::Zero(),  // FR
        Eigen::Vector3d::Zero(),  // RL
        Eigen::Vector3d::Zero()   // RR
    };  // Initialize with 4 zero vectors
};

class SharedQuadrupedInfo {
public:
    static SharedQuadrupedInfo& getInstance();
    static void cleanup();  // Add cleanup method

    // Prevent copying
    SharedQuadrupedInfo(const SharedQuadrupedInfo&) = delete;
    void operator=(const SharedQuadrupedInfo&) = delete;

    // Public members
    pinocchio::Model default_model_;
    QuadrupedState state_;
    BalanceControlInfo balance_;
    RobotParameters robot_params;
    HardwareInfo hardware_;
    bool is_initialized_{false};
    std::mutex mutex_;

private:
    SharedQuadrupedInfo() 
        : default_model_()  // Initialize model first
        , state_()  // Then state with model reference
    {
        // Initialize balance control parameters
        balance_.m = 0.0;
        balance_.g = 9.81;
        balance_.alpha = 0.0;
        balance_.beta = 0.0;
        
        // Initialize all fixed-size arrays first
        balance_.feet_positions.fill(Eigen::Vector3d::Zero());
        
        // Initialize all Eigen matrices explicitly
        balance_.inertia = Eigen::Matrix3d::Zero();
        balance_.Ig = Eigen::Matrix3d::Zero();
        balance_.F_star_prev = Eigen::Matrix<double, 4, 4>::Zero();
        balance_.S = Eigen::Matrix<double, 6, 6>::Zero();
        balance_.C = Eigen::Matrix<double, 3, 4>::Zero();
        balance_.d = Eigen::Matrix<double, 3, 3>::Zero();
        
        // Initialize dynamic-size vectors last
        robot_params.leg_positions = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
    }

    static std::unique_ptr<SharedQuadrupedInfo> instance_;
    static std::once_flag init_flag_;
    static std::mutex cleanup_mutex_;
};

// Note: There may be a segfault during system shutdown due to static destruction order.
// This is expected and does not affect runtime functionality.
extern SharedQuadrupedInfo& quadruped_info;

} // namespace quadruped_mpc
