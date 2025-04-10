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
        int received_bytes = ::read(i2c_fd_, data, length);
        if (received_bytes != static_cast<ssize_t>(length)) {

            std::cerr << "Length requested: "<< length <<"  Length received: " << received_bytes << std::endl;
            std::cerr << "I2C read error" << std::endl;
            return false;
        }
        return true;
    }
    
    bool sendPacket(uint8_t channel, const std::vector<uint8_t>& data) {
    // Create a single buffer for both header and data
    std::vector<uint8_t> complete_packet(SHTP_HEADER_SIZE + data.size());
    uint16_t packet_size = data.size() + SHTP_HEADER_SIZE;
    
    // Construct the header directly in the packet buffer
    complete_packet[0] = packet_size & 0xFF;
    complete_packet[1] = (packet_size >> 8) & 0xFF;
    complete_packet[2] = channel;
    complete_packet[3] = seq_number_[channel]++;
    
    // Copy data to the packet buffer (after header)
    std::copy(data.begin(), data.end(), complete_packet.begin() + SHTP_HEADER_SIZE);
    
    // Send the entire packet in a single I2C transaction
    if (!i2c_write(complete_packet.data(), complete_packet.size())) {
        std::cerr << "Failed to send SHTP packet" << std::endl;
        return false;
    }
    
    // Debug output
    std::cout << "Sent packet: ch=" << (int)channel 
              << " seq=" << (int)(seq_number_[channel]-1)
              << " size=" << packet_size << " bytes" << std::endl;
    
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

    // Report IDs for different features
    #define SENSOR_REPORTID_ACCELEROMETER        0x01
    #define SENSOR_REPORTID_GYROSCOPE            0x02
    #define SENSOR_REPORTID_MAGNETIC_FIELD       0x03
    #define SENSOR_REPORTID_LINEAR_ACCELERATION  0x04
    #define SENSOR_REPORTID_ROTATION_VECTOR      0x05
    #define SENSOR_REPORTID_GRAVITY              0x06
    #define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
    #define SENSOR_REPORTID_STEP_COUNTER         0x11
    #define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
    #define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

    // Channel definitions
    #define CHANNEL_COMMAND  0
    #define CHANNEL_CONTROL  2
    #define CHANNEL_REPORTS  3


    bool setBNO08xFastMode(uint16_t report_interval_ms = 5) {  // 5ms = 200Hz
        // 1. Enable rotation vector reports at high speed
        if (!enableFeature(SENSOR_REPORTID_ROTATION_VECTOR, report_interval_ms)) {
            std::cerr << "Fuck" << std::endl;
            return false;
        }
        
        // 2. Enable accelerometer reports at high speed
        if (!enableFeature(SENSOR_REPORTID_ACCELEROMETER, report_interval_ms)) {
            return false;
        }
        
        // 3. Enable gyroscope reports at high speed
        if (!enableFeature(SENSOR_REPORTID_GYROSCOPE, report_interval_ms)) {
            return false;
        }
        
        // 4. Configure dynamic calibration for fast mode
        if (!setDynamicCalibrationMode(true)) {
            return false;
        }
        
        // Success if we got here
        std::cerr << "fast mode returning true" << std::endl;
        return true;
        
    }

    /**
     * Enable a specific feature report on the BNO08x
     * @param feature_id The feature/report ID to enable
     * @param interval_ms The report interval in milliseconds
     * @return True if successful
     */
    bool enableFeature(uint8_t feature_id, uint16_t interval_ms) {
        // Command to enable a sensor/feature
        std::vector<uint8_t> command = {
            0xFD,                          // Set Feature Command
            feature_id,                    // Feature Report ID
            0,                             // Feature flags (none)
            0, 0,                          // Change sensitivity (not used)
            (uint8_t)(interval_ms & 0xFF), // Report interval LSB
            (uint8_t)((interval_ms >> 8) & 0xFF), // Report interval MSB
            0                              // Maximum batch report interval (not used)
        };
        
        // Send command packet
        if (!sendPacket(CHANNEL_CONTROL, command)) {
            std::cerr << "Failed to enable feature 0x" << std::hex << (int)feature_id << std::dec << std::endl;
            return false;
        }
        
        // Wait for the change to take effect
        usleep(10000);  // 10ms
        
        return true;
    }

    /**
     * Configure dynamic calibration for the BNO08x
     * @param enable Whether to enable (true) or disable (false) dynamic calibration
     * @return True if successful
     */
    bool setDynamicCalibrationMode(bool enable) {
        // Dynamic calibration enables faster response to motion changes
        std::vector<uint8_t> command = {
            0xF2,                   // ME Calibrate command
            0,                      // Subcommand: Set calibration
            (uint8_t)(enable ? 1 : 0), // Enable/disable dynamic calibration
            0                       // Reserved
        };
        
        if (!sendPacket(CHANNEL_CONTROL, command)) {
            std::cerr << "Failed to set dynamic calibration mode" << std::endl;
            return false;
        }
        
        // Wait for the change to take effect
        usleep(50000);  // 50ms
        
        return true;
    }
        
    bool begin(const char* i2c_device = "/dev/i2c-7") {
        // Open I2C device
        i2c_fd_ = open(i2c_device, O_RDWR);
        if (i2c_fd_ < 0) {
            std::cerr << "Failed to open I2C device" << std::endl;
            return false;
        }


        // Get current I2C functionality
        unsigned long funcs;
        ioctl(i2c_fd_, I2C_FUNCS, &funcs);
        
        // Enable fast mode after initialization
        if (!setBNO08xFastMode()) {
            std::cerr << "Warning: Failed to set BNO08x to fast mode" << std::endl;
            // Continue anyway, as we at least have basic functionality
        } else {
            std::cout << "BNO08x configured in fast mode" << std::endl;
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
        
        // // Enable quaternion reports
        // if (!enableQuaternionReports()) {
        //     std::cerr << "Failed to enable quaternion reports" << std::endl;
        //     return false;
        // }
        

        // // Read the product ID, but first we need to send a request
        // // The simplest approach is to just try to read and see if we get any response
        // unsigned char buffer[10];
        
        // if (read(i2c_fd_, buffer, 4) < 0) {
        //     printf("Failed to read from device\n");
        // } else {
        //     printf("Successfully read from device. First 4 bytes: %02X %02X %02X %02X\n", 
        //         buffer[0], buffer[1], buffer[2], buffer[3]);
        // }

        // std::vector<uint8_t> rawData;
        // if (readData(rawData)) {
        //     std::cout << "Successfully read raw data!" << std::endl;
        // } else {
        //     std::cout << "Failed to read raw data" << std::endl;
        // }
        
        // // Try reading quaternions
        // float quaternion[4]; // w, x, y, z
        // if (readQuaternion(quaternion)) {
        //     std::cout << "Quaternion: w=" << quaternion[0]
        //             << ", x=" << quaternion[1]
        //             << ", y=" << quaternion[2]
        //             << ", z=" << quaternion[3] << std::endl;
        // } else {
        //     std::cout << "Failed to read quaternion" << std::endl;
        // }
            
        // // close(i2c_device);


        return true;
    }

    // Simplified read quaternion method - just tries to read raw data
    bool readData(std::vector<uint8_t>& data) {
        uint8_t header[SHTP_HEADER_SIZE];
        
        // Read header
        if (!i2c_read(header, sizeof(header))) {
            std::cout << "Setting FALSE" <<std::endl;
            return false;
        }
        
        // Parse header
        uint16_t packet_size = ((uint16_t)header[1] << 8) | header[0];
        packet_size -= SHTP_HEADER_SIZE; // Remove header size
        uint8_t channel = header[2];
        
        std::cout << "Got packet: size=" << packet_size << ", channel=" << (int)channel << std::endl;
        
        // Read the actual data
        data.resize(packet_size);
        if (packet_size > 0) {
            bool read_success = i2c_read(data.data(), packet_size);
            std::cout << "Read i2c success:" << read_success << std::endl;

            return read_success;
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

    int scan() {
    int file;
    char filename[20];
    
    snprintf(filename, 19, "/dev/i2c-7");
    file = open(filename, O_RDWR);
    if (file < 0) {
        printf("Failed to open I2C bus\n");
        return 1;
    }
    
    printf("Scanning I2C bus...\n");
    
    for (int addr = 0x03; addr < 0x78; addr++) {
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            continue;
        }
        
        // Try to read a byte from the device
        char buf;
        if (read(file, &buf, 1) >= 0) {
            printf("Found device at address 0x%02X\n", addr);
        }
    }
    
    close(file);
    return 0;
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