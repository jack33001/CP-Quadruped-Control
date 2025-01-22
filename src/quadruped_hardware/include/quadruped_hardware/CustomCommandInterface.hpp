#ifndef QUADRUPED_HARDWARE__CUSTOM_COMMAND_INTERFACE_HPP_
#define QUADRUPED_HARDWARE__CUSTOM_COMMAND_INTERFACE_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <array>

namespace quadruped_hardware {

class CustomCommandInterface : public hardware_interface::CommandInterface {
public:
    CustomCommandInterface(const std::string &name, const std::string &interface_name, std::array<uint8_t, 8> *data_ptr)
        : hardware_interface::CommandInterface(name, interface_name), data_ptr_(data_ptr) {}

    void set_value(const std::array<uint8_t, 8> &value) {
        *data_ptr_ = value;
    }

    std::array<uint8_t, 8> get_value() const {
        return *data_ptr_;
    }

private:
    std::array<uint8_t, 8> *data_ptr_;
};

}  // namespace quadruped_hardware

#endif  // QUADRUPED_HARDWARE__CUSTOM_COMMAND_INTERFACE_HPP_