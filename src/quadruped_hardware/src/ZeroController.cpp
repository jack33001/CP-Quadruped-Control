// Controller to zero the robot's joints

#include "ZeroController.h"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <memory>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace my_robot_controllers
{

struct SharedJointData {
  double positions[8];
  double velocities[8];
};

class ZeroController : public controller_interface::ControllerInterface
{
public:
  ZeroController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn on_init() override
  {
    using namespace boost::interprocess;
    try {
      // Create or open shared memory
      shm_ = std::make_unique<managed_shared_memory>(
        open_or_create, "MySharedMemory", 1024 * 1024);

      // Construct our shared data structure
      shared_data_ = shm_->find_or_construct<SharedJointData>("JointData")();
    } catch (...) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to open or create shared memory.");
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    // Load joint names or other parameters if needed
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // Example for writing to shared memory
    // Here we assume 8 joints
    for (int i = 0; i < 8; i++) {
      // Fake data: zero everything
      shared_data_->positions[i] = 0.0;
      shared_data_->velocities[i] = 0.0;
    }
    return controller_interface::return_type::OK;
  }

private:
  std::unique_ptr<boost::interprocess::managed_shared_memory> shm_;
  SharedJointData* shared_data_;
};

}  // namespace my_robot_controllers