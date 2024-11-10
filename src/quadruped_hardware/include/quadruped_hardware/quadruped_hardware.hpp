// hardware_interface.hpp
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>

namespace quadruped_hardware {

class QuadrupedHardware : public hardware_interface::SystemInterface {
public:
    QuadrupedHardware();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Hardware communication objects
    struct CanBus {
        // Add your CAN bus implementation details
        bool initialize(const std::string& interface);
        void write(int id, const std::vector<uint8_t>& data);
        std::vector<uint8_t> read(int id);
    };

    struct I2CBus {
        // Add your I2C bus implementation details
        bool initialize(const std::string& interface);
        void write(int address, const std::vector<uint8_t>& data);
        std::vector<uint8_t> read(int address);
    };

    // Hardware components
    struct Motor {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
        double position_command{0.0};
        double velocity_command{0.0};
        double effort_command{0.0};
        int can_id;
    };

    struct IMU {
        double orientation[4]{0.0};  // quaternion
        double angular_velocity[3]{0.0};
        double linear_acceleration[3]{0.0};
        int i2c_address;
    };

    struct PowerBoard {
        double voltage{0.0};
        double current{0.0};
        int can_id;
    };

    // Member variables
    std::vector<Motor> motors_;
    IMU imu_;
    PowerBoard power_board_;
    CanBus can_bus_;
    I2CBus i2c_bus_;

    // Configuration parameters
    std::string can_interface_;
    std::string i2c_interface_;
};

} // namespace quadruped_hardware