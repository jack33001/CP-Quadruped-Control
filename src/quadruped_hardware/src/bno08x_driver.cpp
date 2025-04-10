#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <iostream>

// BNO08x constants
#define BNO08X_I2C_ADDR 0x4A  // Default I2C address (some modules use 0x4B)

// SHTP Header size
#define SHTP_HEADER_SIZE 4

// Channel IDs
#define CHANNEL_COMMAND 0
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 5

// Report IDs
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05

// Commands
#define COMMAND_ME_CALIBRATE 0xF2
#define COMMAND_PRODUCT_ID_REQ 0xF9
#define COMMAND_FRS_WRITE 0xF7
#define COMMAND_FRS_READ_REQ 0xF8

class BNO08x {
private:
    int i2c_fd_;
    uint8_t seq_number_[6] = {0, 0, 0, 0, 0, 0}; // Sequence number for each channel
    
    // I2C write wrapper
    bool i2c_write(const uint8_t* data, size_t length) {
        if (::write(i2c_fd_, data, length) != static_cast<ssize_t>(length)) {
            std::cerr << "I2C write error" << std::endl;
            return false;
        }
        return true;
    }
    
    // I2C read wrapper
    bool i2c_read(uint8_t* data, size_t length) {
        if (::read(i2c_fd_, data, length) != static_cast<ssize_t>(length)) {
            std::cerr << "I2C read error" << std::endl;
            return false;
        }
        return true;
    }
    
    // Send packet through specified channel
    bool sendPacket(uint8_t channel, const std::vector<uint8_t>& data) {
        uint8_t header[SHTP_HEADER_SIZE];
        uint16_t packet_size = data.size() + SHTP_HEADER_SIZE;
        
        // Construct the header
        header[0] = packet_size & 0xFF;
        header[1] = (packet_size >> 8) & 0xFF;
        header[2] = channel;
        header[3] = seq_number_[channel]++;
        
        // Send header first
        if (!i2c_write(header, SHTP_HEADER_SIZE)) {
            return false;
        }
        
        // Then send the data
        if (!i2c_write(data.data(), data.size())) {
            return false;
        }
        
        return true;
    }
    
    // Read available packet
    bool receivePacket(uint8_t* channel, std::vector<uint8_t>& data) {
        uint8_t header[SHTP_HEADER_SIZE];
        
        // Read header
        if (!i2c_read(header, SHTP_HEADER_SIZE)) {
            return false;
        }
        
        // Parse header
        uint16_t packet_size = ((uint16_t)header[1] << 8) | header[0];
        packet_size -= SHTP_HEADER_SIZE; // Remove header size
        *channel = header[2];
        
        // Read the actual data
        data.resize(packet_size);
        if (!i2c_read(data.data(), packet_size)) {
            return false;
        }
        
        return true;
    }
    
    // Enable quaternion reports
    bool enableQuaternionReports(uint16_t interval_ms = 10) { // 10ms = 100Hz
        std::vector<uint8_t> command = {
            SENSOR_REPORTID_ROTATION_VECTOR, // Report ID
            0,                               // Feature flags
            0,                               // Change sensitivity LSB
            0,                               // Change sensitivity MSB
            (uint8_t)(interval_ms & 0xFF),   // Report interval LSB
            (uint8_t)((interval_ms >> 8) & 0xFF), // Report interval MSB
            0                                // Batch interval
        };
        
        return sendPacket(CHANNEL_CONTROL, command);
    }
    
public:
    BNO08x() : i2c_fd_(-1) {}
    
    ~BNO08x() {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
        }
    }
    
    bool begin(const char* i2c_device = "/dev/i2c-7") {
        // Open I2C device
        i2c_fd_ = open(i2c_device, O_RDWR);
        if (i2c_fd_ < 0) {
            std::cerr << "Failed to open I2C device" << std::endl;
            return false;
        }
        
        // Set I2C slave address
        if (ioctl(i2c_fd_, I2C_SLAVE, BNO08X_I2C_ADDR) < 0) {
            std::cerr << "Failed to set I2C slave address" << std::endl;
            close(i2c_fd_);
            i2c_fd_ = -1;
            return false;
        }
        
        // Reset sequence numbers
        for (int i = 0; i < 6; i++) {
            seq_number_[i] = 0;
        }
        
        // Small delay for sensor to boot
        usleep(100000); // 100ms
        
        // Enable quaternion reports
        if (!enableQuaternionReports()) {
            std::cerr << "Failed to enable quaternion reports" << std::endl;
            return false;
        }
        
        return true;
    }
    
    // Read quaternion data
    bool readQuaternion(float* quat) {
        uint8_t channel;
        std::vector<uint8_t> data;
        
        // Try to read packets until we get quaternion data
        for (int attempt = 0; attempt < 10; attempt++) {
            if (receivePacket(&channel, data)) {
                // Check if this is a report packet
                if (channel == CHANNEL_REPORTS && data.size() >= 21) {
                    // Check if this is a quaternion report
                    if (data[0] == SENSOR_REPORTID_ROTATION_VECTOR) {
                        // Extract quaternion components (Q point format)
                        int16_t i = (int16_t)((data[2] << 8) | data[1]);
                        int16_t j = (int16_t)((data[4] << 8) | data[3]);
                        int16_t k = (int16_t)((data[6] << 8) | data[5]);
                        int16_t real = (int16_t)((data[8] << 8) | data[7]);
                        
                        // Convert to float (Q point 14)
                        const float qpoint_14_factor = 1.0f / (1 << 14);
                        quat[0] = real * qpoint_14_factor;  // w
                        quat[1] = i * qpoint_14_factor;     // x
                        quat[2] = j * qpoint_14_factor;     // y
                        quat[3] = k * qpoint_14_factor;     // z
                        
                        return true;
                    }
                }
            }
            
            // Wait a bit before trying again
            usleep(5000); // 5ms
        }
        
        return false;
    }
};

// // Example usage
// int main() {
//     BNO08x sensor;
    
//     if (!sensor.begin()) {
//         std::cerr << "Failed to initialize BNO08x" << std::endl;
//         return 1;
//     }
    
//     std::cout << "BNO08x initialized successfully" << std::endl;
    
//     // Main loop to read quaternion data
//     while (true) {
//         float quaternion[4]; // w, x, y, z
        
//         if (sensor.readQuaternion(quaternion)) {
//             std::cout << "Quaternion: w=" << quaternion[0]
//                       << ", x=" << quaternion[1]
//                       << ", y=" << quaternion[2]
//                       << ", z=" << quaternion[3] << std::endl;
//         } else {
//             std::cout << "Failed to read quaternion" << std::endl;
//         }
        
//         usleep(100000); // 100ms
//     }
    
//     return 0;
// }