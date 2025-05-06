#ifndef QUADRUPED_HARDWARE_HEX_UTILS_HPP_
#define QUADRUPED_HARDWARE_HEX_UTILS_HPP_

#include <string>
#include <cstdint>


namespace quadruped_hardware {
// Converts an 8-byte hexadecimal string to a double
double hexStringToDouble(const std::string& hexStr);

// Converts a double to an 8-byte hexadecimal string
std::string doubleToHexString(double value);

} // namespace quadruped_hardware
#endif // HEX_UTILS_HPP
