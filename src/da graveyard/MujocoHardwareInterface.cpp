#include "quadruped/MujocoHardwareInterface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>

namespace quadruped
{

MujocoHardwareInterface::MujocoHardwareInterface() : window(nullptr), mj_model(nullptr), mj_data(nullptr) {
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_init");

    node->declare_parameter("mujoco_model_path", "");
    node->declare_parameter("urdf_path", "");

    std::string mujoco_model_path = node->get_parameter("mujoco_model_path").as_string();
    std::string urdf_path = node->get_parameter("urdf_path").as_string();

    if (mujoco_model_path.empty() || urdf_path.empty()) {
        RCLCPP_ERROR(node->get_logger(), "MuJoCo model path or URDF path is not specified.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node->get_logger(), "Loading MuJoCo model from: %s", mujoco_model_path.c_str());
    mj_model = mj_loadModel(mujoco_model_path.c_str(), nullptr);
    if (!mj_model) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load MuJoCo model from file: %s", mujoco_model_path.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node->get_logger(), "Creating MuJoCo data");
    mj_data = mj_makeData(mj_model);
    if (!mj_data) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize MuJoCo data.");
        mj_deleteModel(mj_model);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node->get_logger(), "Initializing state and command vectors");
    hw_states_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_efforts_.resize(info_.joints.size(), 0.0);

    RCLCPP_INFO(node->get_logger(), "Initializing MuJoCo visualization");
    mjv_defaultOption(&vopt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_defaultCamera(&cam);

    if (!glfwInit()) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize GLFW");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize control inputs to zero
    RCLCPP_INFO(node->get_logger(), "Initializing control inputs to zero");
    mju_zero(mj_data->ctrl, mj_model->nu);

    // Reset the simulation to a stable initial state
    mj_resetData(mj_model, mj_data);
    
    // Adjust camera settings
    cam.type = mjCAMERA_FREE;
    cam.trackbodyid = 1;
    cam.distance = 4.0;
    cam.azimuth = 90.0;
    cam.elevation = -20.0;

    // Set up the scene
    mjv_defaultScene(&scn);
    mjv_makeScene(mj_model, &scn, 2000);

    // Adjust scene settings
    scn.enabletransform = 1;
    scn.scale = 1.0;
    scn.stereo = mjSTEREO_NONE;

    // These are part of mjvGLCamera, which is automatically 
    // handled by mjv_updateScene
    // We don't need to set them manually, but if needed:
    // scn.camera[0].frustum_near = 0.1;
    // scn.camera[0].frustum_far = 100.0;

    // Update the scene with our camera settings
    mjv_updateScene(mj_model, mj_data, &vopt, NULL, &cam, mjCAT_ALL, &scn);

    RCLCPP_INFO(node->get_logger(), "MujocoHardwareInterface initialized successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_configure");
    RCLCPP_INFO(node->get_logger(), "Configuring...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_configure");
    RCLCPP_INFO(node->get_logger(), "Activating...");
    
    window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create GLFW window");
        return hardware_interface::CallbackReturn::ERROR;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_makeScene(mj_model, &scn, 2000);
    mjr_makeContext(mj_model, &con, mjFONTSCALE_150);

    // Reinitialize control inputs to zero
    RCLCPP_INFO(node->get_logger(), "Reinitializing control inputs to zero");
    mju_zero(mj_data->ctrl, mj_model->nu);

    // Reset the simulation to a stable initial state
    mj_resetData(mj_model, mj_data);

    RCLCPP_INFO(node->get_logger(), "Activation complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_deactivate");
    RCLCPP_INFO(node->get_logger(), "Deactivating...");

    RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Cleaning up MujocoHardwareInterface");
    
    // Clean up GLFW window and MuJoCo scene and context
    if (window)
    {
        glfwDestroyWindow(window);
        window = nullptr;
    }

    if (mj_data)
    {
        mj_deleteData(mj_data);
        mj_data = nullptr;
    }

    if (mj_model)
    {
        mj_deleteModel(mj_model);
        mj_model = nullptr;
    }

    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    RCLCPP_INFO(node->get_logger(), "MuJoCo scene and context freed.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoHardwareInterface::export_state_interfaces()
{
    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_esi");
    RCLCPP_INFO(node->get_logger(), "Exporting state interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoHardwareInterface::export_command_interfaces()
{
    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_eci");
    RCLCPP_INFO(node->get_logger(), "Exporting command interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Export the position command interface
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));

        // Export the effort command interface
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type MujocoHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_read");
    RCLCPP_INFO(node->get_logger(), "Reading...");
    for (size_t i = 0; i < info_.joints.size(); i++) {
        hw_states_[i] = mj_data->qpos[i];
        hw_velocities_[i] = mj_data->qvel[i];
        hw_efforts_[i] = mj_data->qfrc_applied[i];
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto node = std::make_shared<rclcpp::Node>("mujoco_hardware_interface_write");
    RCLCPP_INFO(node->get_logger(), "Writing...");
    for (size_t i = 0; i < info_.joints.size(); i++) {
        mj_data->ctrl[i] = hw_commands_[i];
    }

    mj_step(mj_model, mj_data);

    if (window && !glfwWindowShouldClose(window)) {
        mjv_updateScene(mj_model, mj_data, &vopt, NULL, &cam, mjCAT_ALL, &scn);
        
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        
        mjrRect viewport = {0, 0, width, height};
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    return hardware_interface::return_type::OK;
}

MujocoHardwareInterface::~MujocoHardwareInterface() {
    RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Destroying MujocoHardwareInterface");
    
    if (mj_data) {
        mj_deleteData(mj_data);
        mj_data = nullptr; // Set to nullptr after deletion
    }

    if (mj_model) {
        mj_deleteModel(mj_model);
        mj_model = nullptr; // Set to nullptr after deletion
    }

    if (window) {
        glfwDestroyWindow(window);
        window = nullptr; // Set to nullptr after deletion
    }

    glfwTerminate(); // Call terminate, no need to check again
}

}  // namespace quadruped

PLUGINLIB_EXPORT_CLASS(
  quadruped::MujocoHardwareInterface,
  hardware_interface::SystemInterface
)