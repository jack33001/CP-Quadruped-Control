#include "quadruped_hardware/i2c_device.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>
#include <thread>
#include <stdexcept>

const uint8_t kDefaultDeviceAddress = 0x4A;
const uint8_t kMode1Reg = 0x00;
const uint8_t kMode2Reg = 0x01;
const uint8_t kPrescaleReg = 0xFE;
const uint8_t kPwmReg = 0x06;
const float kReferenceClockSpeed = 25e6;
const float kPwmFrequency = 1600.0;

using namespace std::chrono_literals;
namespace JetBotControl{

I2CDevice::I2CDevice() {
  // i2c_fd_ = open("/dev/i2c-7", O_RDWR);
  // if (i2c_fd_ < 0) {
  //   throw std::runtime_error("Failed to open I2C interface!");
  // }


  // if(ioctl(i2c_fd_, I2C_SLAVE_FORCE, kDefaultDeviceAddress) < 0){
  //   close(i2c_fd_);
  //   throw std::runtime_error("Failed to set slave address!");
    
  // }

  // int i2c_bus;
  // const char *filename = "/dev/i2c-7"; // Replace with your I2C bus device
  // int i2c_address = 0x4a; // Replace with the I2C address of your device
  // char buf[2] = {0};

  // 1. Open the I2C bus

  if ((i2c_bus = open(filename, O_RDWR)) < 0) {
      perror("Failed to open the i2c bus");
      exit(1);
  }

  // 2. Set the I2C slave address
  if (ioctl(i2c_bus, I2C_SLAVE, i2c_address) < 0) {
      printf("Failed to acquire bus access and/or talk to slave.\n");
      close(i2c_bus);
      exit(1);
  }

  // // Select the PWM device
  // if (!trySelectDevice()){
  //   throw std::runtime_error("Failed to select device!");
  // }
    

  // // Reset the PWM device
  // if (!tryReset())  throw std::runtime_error("Failed to reset PWM device!");

  // // Set the PWM device clock
  // if (!trySetClock())  throw std::runtime_error("Failed to set PWM clock!");
}

I2CDevice::~I2CDevice() {
  tryReset();
  if (i2c_bus) {
    close(i2c_bus);
  }
}

bool I2CDevice::enable_feature(uint8_t feature_id,int freq) {
  uint8_t set_feature_report[17]= {0x00};

  set_feature_report[0] = bno_msgs::SET_FEATURE_COMMAND;
  set_feature_report[1] = feature_id;

  return write(i2c_bus, buf_, 17) == 17;
}

bool I2CDevice::tryEnableMotor(uint8_t pin) {
  buf_[0] = kPwmReg + 4 * pin;
  buf_[1] = 0x00;
  buf_[2] = 0x10;
  buf_[3] = 0x00;
  buf_[4] = 0x00;
  return write(i2c_bus, buf_, 5) == 5;
}

int I2CDevice::i2c_write_array(const uint8_t* data, int length) {
    // Ensure the data pointer is valid and length is positive
    if (data == nullptr || length <= 0) {
        throw std::invalid_argument("Invalid data pointer or length");
    }

    // Write the byte array to the I2C bus
    int bytes_written = ::write(i2c_bus, data, length);

    // Check if the number of bytes written matches the expected length
    if (bytes_written != length) {
        throw std::runtime_error("Failed to write the complete byte array to the I2C device");
    }

    return bytes_written;
}


bool I2CDevice::trySetDutyCycle(uint8_t pin, uint16_t duty_cycle) {
  buf_[0] = kPwmReg + 4 * pin;

  if (duty_cycle == 0xFFFF) {
    // Special case - fully on
    buf_[1] = 0x00;
    buf_[2] = 0x10;
    buf_[3] = 0x00;
    buf_[4] = 0x00;
  } else if (duty_cycle < 0x0010) {
    // Special case - fully off
    buf_[1] = 0x00;
    buf_[2] = 0x00;
    buf_[3] = 0x00;
    buf_[4] = 0x10;
  } else {
    // Shift by 4 to fit 12-bit register
    uint16_t value = duty_cycle >> 4;
    buf_[1] = 0x00;
    buf_[2] = 0x00;
    buf_[3] = value & 0xFF;
    buf_[4] = (value >> 8) & 0xFF;
  }

  return write(i2c_bus, buf_, 5) == 5;
}

bool I2CDevice::tryWriteReg(uint8_t reg, uint8_t data) {
  buf_[0] = reg;
  buf_[1] = data;
  return write(i2c_bus, buf_, 2) == 2;
}
std::optional<uint8_t> I2CDevice::tryReadReg(uint8_t reg) {
  buf_[0] = reg;
  if (write(i2c_bus, buf_, 1) != 1) {
    return std::nullopt;
  }
  if (read(i2c_bus, buf_, 1) != 1) {
    return std::nullopt;
  }
  return buf_[0];
}

bool I2CDevice::trySetClock() {
  const uint8_t prescale =
      (kReferenceClockSpeed / 4096.0 / kPwmFrequency + 0.5) - 1;
  if (prescale < 3) return false;

  // Read old mode
  const auto old_mode_op = tryReadReg(kMode1Reg);
  if (!old_mode_op.has_value()) return false;
  const uint8_t old_mode = *old_mode_op;

  // Set a new mode/prescaler, then sleep
  const uint8_t new_mode = (old_mode & 0x7F) | 0x10;
  if (!tryWriteReg(kMode1Reg, new_mode)) return false;
  if (!tryWriteReg(kPrescaleReg, prescale)) return false;
  if (!tryWriteReg(kMode1Reg, old_mode)) return false;
  std::this_thread::sleep_for(5ms);

  // Set mode 1 with autoincrement, fix to stop pca9685 from accepting commands
  // at all addresses
  if (!tryWriteReg(kMode1Reg, old_mode | 0xA0)) return false;

  return true;
}

bool I2CDevice::tryReset() { return tryWriteReg(kMode1Reg, 0x00); }

bool I2CDevice::trySelectDevice() {
  return ioctl(i2c_bus, I2C_SLAVE, kDefaultDeviceAddress) >= 0;
}


// namespace bno_msgs{ 
//   // Configuring Reports;
//   uint8_t COMMAND_REQUEST = 0xF2;
//   uint8_t FRS_READ_RESPONSE = 0xF3;
//   uint8_t FRS_READ_REQUEST = 0xF4;
//   uint8_t FRS_WRITE_RESPONSE = 0xF5;
//   uint8_t FRS_WRITE_DATA = 0xF6;
//   uint8_t FRS_WRITE_REQUEST = 0xF7;
//   uint8_t SHTP_REPORT_ID_RESPONSE = 0xF8;
//   uint8_t SHTP_REPORT_ID_REQUEST = 0xF9;
//   uint8_t REBASE_TIMESTAMP = 0xFA;
//   uint8_t BASE_TIMESTAMP = 0xFB;
//   uint8_t GET_FEATURE_RESPONSE = 0xFC;
//   uint8_t SET_FEATURE_COMMAND = 0xFD;
//   uint8_t GET_FEATURE_REQUEST = 0xFE;
//   // DCD/ ME Commands
//   uint8_t ME_ERRORREPORT_CDE = 0x01;
//   uint8_t ME_COUNTER_CDE = 0x02;
//   uint8_t ME_TARE_CDE = 0x03;
//   uint8_t ME_INIT_CDE = 0x04;
//   uint8_t ME_SAVE_DCD_CDE = 0x06;
//   uint8_t ME_CALIBRATION_CDE = 0x07;
//   uint8_t ME_SAVE_DCD_PERIODIC_CDE = 0x09;
//   uint8_t ME_OSCILLATOR_TYPE_CDE = 0x0A;
//   uint8_t ME_RESET_DCD_CDE = 0x0B;
//   // DCD/ME Sub-commands
//   uint8_t ME_COUNTER_GETCOUNTS_CDE = 0x00;
//   uint8_t ME_COUNTER_CLEARCOUNTS_CDE = 0x01;
//   uint8_t ME_TARE_NOW_SUBCDE = 0x00;
//   uint8_t ME_TARE_PERSIST_SUBCDE = 0x01;
//   uint8_t ME_TARE_REORIENTATION_SUBCDE = 0x02;
//   uint8_t ME_CALIBRATION_CONFIG_SUBCDE = 0x00;
//   uint8_t ME_CALIBRATION_GETCAL_SUBCDE = 0x01;
//   uint8_t ME_SAVE_DCD_PERIODIC_ENABLE_SUBCDE = 0x00;
//   uint8_t ME_SAVE_DCD_PERIODIC_DISABLE_SUBCDE = 0x00;
//   // BNO CONFIGURATION RECORDS;
//   int BNO_CONF_STATIC_CALIBRATION_AGM = 0x7979;
//   int BNO_CONF_NOMINAL_CALIBRATION_AGM = 0x4D4D;
//   int BNO_CONF_STATIC_CALIBRATION_SRA = 0x8A8A;
//   int BNO_CONF_NOMINAL_CALIBRATION_SRA = 0x4E4E;
//   int BNO_CONF_DYNAMIC_CALIBRATION = 0x1F1F;
//   int BNO_CONF_MOTION_ENGINE_POWER_MANAGEMENT = 0xD3E2;
//   int BNO_CONF_SYSTEM_ORIENTATION = 0x2D3E;
//   int BNO_CONF_PRIMARY_ACCELEROMETER_ORIENTATION = 0x2D41;
//   int BNO_CONF_SCREEN_ROTATION_ACCELEROMETER_ORIENTATION = 0x2D43;
//   int BNO_CONF_GYROSCOPE_ORIENTATION = 0x2D46;
//   int BNO_CONF_MAGNETOMETER_ORIENTATION = 0x2D4C;
//   int BNO_CONF_ARVR_STABILIZATION_ROTATION_VECTOR = 0x3E2D;
//   int BNO_CONF_ARVR_STABILIZATION_GAME_ROTATION_VECTOR = 0x3E2E;
//   int BNO_CONF_SIGNIFICANT_MOTION_DETECTOR = 0xC274;
//   int BNO_CONF_SHAKE_DETECTOR = 0x7D7D;
//   int BNO_CONF_MAXIMUM_FUSION_PERIOD = 0xD7D7;
//   int BNO_CONF_SERIAL_NUMBER = 0x4B4B;
//   int BNO_CONF_ENV_SENSOR_PRESSURE_CALIBRATION = 0x39AF;
//   int BNO_CONF_ENV_SENSOR_TEMPERATURE_CALIBRATION = 0x4D20;
//   int BNO_CONF_ENV_SENSOR_HUMIDITY_CALIBRATION = 0x1AC9;
//   int BNO_CONF_ENV_SENSOR_AMBIENT_LIGHT_CALIBRATION = 0x39B1;
//   int BNO_CONF_ENV_SENSOR_PROXIMITY_CALIBRATION = 0x4DA2;
//   int BNO_CONF_ALS_CALIBRATION = 0xD401;
//   int BNO_CONF_PROXIMITY_SENSOR_CALIBRATION = 0xD402;
//   int BNO_CONF_PICKUP_DETECTOR = 0x1B2A;
//   int BNO_CONF_FLIP_DETECTOR = 0xFC94;
//   int BNO_CONF_STABILITY_DETECTOR = 0xED85;
//   int BNO_CONF_ACTIVITY_TRACKER = 0xED88;
//   int BNO_CONF_SLEEP_DETECTOR = 0xED87;
//   int BNO_CONF_TILT_DETECTOR = 0xED89;
//   int BNO_CONF_POCKET_DETECTOR = 0xEF27;
//   int BNO_CONF_CIRCLE_DETECTOR = 0xEE51;
//   int BNO_CONF_USER_RECORD = 0x74B4;
//   int BNO_CONF_MOTION_ENGINE_TIME_SOURCE_SELECTION = 0xD403;
//   int BNO_CONF_UART_OUTPUT_FORMAT_SELECTION = 0xA1A1;
//   int BNO_CONF_GYRO_INTEGRATED_ROTATION_VECTOR = 0xA1A2;
//   int BNO_CONF_FUSION_CONTROL_FLAGS = 0xA1A3;
//   // Reports Summary depending on BNO device ;
//   uint8_t BNO_REPORT_ACCELEROMETER = 0x01;
//   uint8_t BNO_REPORT_GYROSCOPE = 0x02;
//   uint8_t BNO_REPORT_MAGNETOMETER = 0x03;
//   uint8_t BNO_REPORT_LINEAR_ACCELERATION = 0x04;
//   uint8_t BNO_REPORT_ROTATION_VECTOR = 0x05;
//   uint8_t BNO_REPORT_GRAVITY = 0x06;
//   uint8_t BNO_REPORT_UNCALIBRATED_GYROSCOPE = 0x07;
//   uint8_t BNO_REPORT_GAME_ROTATION_VECTOR = 0x08;
//   uint8_t BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09;
//   uint8_t BNO_REPORT_PRESSURE = 0x0A;
//   uint8_t BNO_REPORT_AMBIENT_LIGHT = 0x0B;
//   uint8_t BNO_REPORT_HUMIDITY = 0x0C;
//   uint8_t BNO_REPORT_PROXIMITY = 0x0D;
//   uint8_t BNO_REPORT_TEMPERATURE = 0x0E;
//   uint8_t BNO_REPORT_UNCALIBRATED_MAGNETOMETER = 0x0F;
//   uint8_t BNO_REPORT_TAP_DETECTOR = 0x10;
//   uint8_t BNO_REPORT_STEP_COUNTER = 0x11;
//   uint8_t BNO_REPORT_SIGNIFICANT_MOTION = 0x12;
//   uint8_t BNO_REPORT_STABILITY_CLASSIFIER = 0x13;
//   uint8_t BNO_REPORT_RAW_ACCELEROMETER = 0x14;
//   uint8_t BNO_REPORT_RAW_GYROSCOPE = 0x15;
//   uint8_t BNO_REPORT_RAW_MAGNETOMETER = 0x16;
//   uint8_t BNO_REPORT_SAR = 0x17;
//   uint8_t BNO_REPORT_STEP_DETECTOR = 0x18;
//   uint8_t BNO_REPORT_SHAKE_DETECTOR = 0x19;
//   uint8_t BNO_REPORT_FLIP_DETECTOR = 0x1A;
//   uint8_t BNO_REPORT_PICKUP_DETECTOR = 0x1B;
//   uint8_t BNO_REPORT_STABILITY_DETECTOR = 0x1C;
//   uint8_t BNO_REPORT_ACTIVITY_CLASSIFIER = 0x1E;
//   uint8_t BNO_REPORT_SLEEP_DETECTOR = 0x1F;
//   uint8_t BNO_REPORT_TILT_DETECTOR = 0x20;
//   uint8_t BNO_REPORT_POCKET_DETECTOR = 0x21;
//   uint8_t BNO_REPORT_CIRCLE_DETECTOR = 0x22;
//   uint8_t BNO_REPORT_HEART_RATE_MONITOR = 0x23;
//   uint8_t BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR = 0x28;
//   uint8_t BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR = 0x29;
//   uint8_t BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A;
//   // Timeouts (ms);
//   int QUAT_READ_TIMEOUT = 500;
//   int PACKET_READ_TIMEOUT = 2000;
//   int FEATURE_ENABLE_TIMEOUT = 2000;
//   int DEFAULT_TIMEOUT = 2000;
// }
}
