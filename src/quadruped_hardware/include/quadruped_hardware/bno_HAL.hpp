#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <iostream>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "quadruped_hardware/sh2_err.h"
#include "quadruped_hardware/sh2.h"

#include "quadruped_hardware/sh2_hal.h"
// #include "quadruped_hardware/bno08x_driver.h"


// namespace quadruped_hardware {
// Define the HAL structure for Jetson
typedef struct {
    int someJetsonSpecificField; // Placeholder for Jetson-specific fields (e.g., I2C/SPI setup)
} JetsonHal_t;

// HAL functions
int jetsonOpen(sh2_Hal_t *self) {
    // Initialize Jetson I2C or SPI communication


    // int i2c_fd_;
    // const char* i2c_device = "/dev/i2c-7"; // Replace with your I2C device path
    // //  Open I2C device
    // i2c_fd_ = open(i2c_device, O_RDWR);
    // if (i2c_fd_ < 0) {
    //     std::cerr << "Failed to open I2C device" << std::endl;
    //     return false;
    // }

    // // Get current I2C functionality
    // unsigned long funcs;
    // ioctl(i2c_fd_, I2C_FUNCS, &funcs);
    
    
    // // Set I2C slave address
    // if (ioctl(i2c_fd_, I2C_SLAVE, BNO08X_I2C_ADDR) < 0) {
    //     std::cerr << "Failed to set I2C slave address" << std::endl;
    //     close(i2c_fd_);
    //     i2c_fd_ = -1;
    //     return false;
    // }
    
    // // Small delay for sensor to boot
    // usleep(100000); // 100ms
        
    // printf("Jetson-specific HAL open called.\n");

    // Add your Jetson-specific initialization here
    return 0; // Replace with error codes if needed
}

void jetsonClose(sh2_Hal_t *self) {
    printf("Jetson-specific HAL close called.\n");
    // Add your Jetson-specific cleanup code here
}

int jetsonRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    // Read data from the BNO08x over I2C or SPI
    printf("Jetson-specific HAL read called.\n");
    return 0; // Replace with the number of bytes read
}

int jetsonWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    // Write data to the BNO08x over I2C or SPI
    printf("Jetson-specific HAL write called.\n");
    return len; // Replace with the number of bytes written
}

uint32_t jetsonGetTimeUs(sh2_Hal_t *self) {
    // Return the current time in microseconds
    return (uint32_t)(clock() * 1000000L / CLOCKS_PER_SEC);
}


bool bnostart() {

    // Create a HAL instance
    static JetsonHal_t jetsonHalInstance;
    static sh2_Hal_t hal = {
        .open = jetsonOpen,
        .close = jetsonClose,
        .read = jetsonRead,
        .write = jetsonWrite,
        .getTimeUs = jetsonGetTimeUs,
    };

    // Initialize the SH2 sensor hub
    if (sh2_open(&hal, NULL, NULL) == SH2_ERR) {
        fprintf(stderr, "Failed to open SH2 sensor hub.\n");
        return EXIT_FAILURE;
    }

    printf("SH2 sensor hub initialized successfully.\n");

    // Example: Enable a sensor (e.g., accelerometer) and read its data
    sh2_SensorConfig_t config = {
        .changeSensitivityEnabled = false,
        .changeSensitivity = 0,
        .reportInterval_us = 100000, // 100ms
        .batchInterval_us = 0,
        .sensorSpecific = 0,
    };

    if (sh2_setSensorConfig(SH2_ACCELEROMETER, &config) != SH2_OK) {
        fprintf(stderr, "Failed to configure accelerometer.\n");
        sh2_close();
        return EXIT_FAILURE;
    }

    printf("Accelerometer enabled.\n");

    // Service the sensor hub (this would typically run in a loop)
    for (int i = 0; i < 10; i++) {
        sh2_service(); // Ensure to call this periodically
        printf("Servicing sensor hub...\n");
        usleep(100000); // Sleep for 100ms
    }

    // Clean up
    sh2_close();
    printf("SH2 sensor hub closed.\n");

 return true;
}

// // Main example
// int main() {


//     // Create a HAL instance
//     static JetsonHal_t jetsonHalInstance;
//     static sh2_Hal_t hal = {
//         .open = jetsonOpen,
//         .close = jetsonClose,
//         .read = jetsonRead,
//         .write = jetsonWrite,
//         .getTimeUs = jetsonGetTimeUs,
//     };

//     // Initialize the SH2 sensor hub
//     if (sh2_open(&hal, NULL, NULL) = SH2_ERR) {
//         fprintf(stderr, "Failed to open SH2 sensor hub.\n");
//         return EXIT_FAILURE;
//     }

//     printf("SH2 sensor hub initialized successfully.\n");

//     // Example: Enable a sensor (e.g., accelerometer) and read its data
//     sh2_SensorConfig_t config = {
//         .changeSensitivityEnabled = false,
//         .changeSensitivity = 0,
//         .reportInterval_us = 100000, // 100ms
//         .batchInterval_us = 0,
//         .sensorSpecific = 0,
//     };

//     if (sh2_setSensorConfig(SH2_ACCELEROMETER, &config) != SH2_OK) {
//         fprintf(stderr, "Failed to configure accelerometer.\n");
//         sh2_close();
//         return EXIT_FAILURE;
//     }

//     printf("Accelerometer enabled.\n");

//     // Service the sensor hub (this would typically run in a loop)
//     for (int i = 0; i < 10; i++) {
//         sh2_service(); // Ensure to call this periodically
//         printf("Servicing sensor hub...\n");
//         usleep(100000); // Sleep for 100ms
//     }

//     // Clean up
//     sh2_close();
//     printf("SH2 sensor hub closed.\n");

//     return EXIT_SUCCESS;
// }

// } // namespace quadruped_hardware