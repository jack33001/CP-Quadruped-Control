#include "quadruped_hardware/hex_utils.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>


namespace quadruped_hardware {
// Converts an 8-byte hexadecimal string to a double
double hexStringToDouble(const std::string& hexStr) {
    if (hexStr.size() != 16) {
        throw std::invalid_argument("Hex string must be exactly 16 characters long.");
    }

    // Convert the hex string to a 64-bit integer
    uint64_t intValue = 0;
    std::stringstream ss(hexStr);
    ss >> std::hex >> intValue;

    // Use a union to reinterpret the bits as a double
    union {
        uint64_t intValue;
        double doubleValue;
    } converter;

    converter.intValue = intValue;
    return converter.doubleValue;
}

// Converts a double to an 8-byte hexadecimal string
std::string doubleToHexString(double value) {
    // Use a union to reinterpret the bits of the double as a 64-bit integer
    union {
        uint64_t intValue;
        double doubleValue;
    } converter;

    converter.doubleValue = value;

    // Convert the 64-bit integer to a hexadecimal string
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0') << std::setw(16) << converter.intValue;

    return ss.str();
}



} // namespace quadruped_hardware