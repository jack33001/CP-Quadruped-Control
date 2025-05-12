#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP


#pragma once

#include <cstdint>
#include <optional>

namespace JetBotControl {

class I2CDevice {
 public:
  I2CDevice();
  ~I2CDevice();
  bool tryEnableMotor(uint8_t pin);
  bool trySetDutyCycle(uint8_t pin, uint16_t duty_cycle);

  bool enable_feature(uint8_t feature_id, int freq);
  bool set_quaternion_euler_vector(uint8_t feature_id);

  int i2c_write_array(const uint8_t* data, int length);

 
  bool trySelectDevice();
  bool tryWriteReg(uint8_t reg, uint8_t data);
  std::optional<uint8_t> tryReadReg(uint8_t reg);
  private:
  
  bool trySetClock();
  bool tryReset();
  int i2c_fd_;
  uint8_t buf_[17]= {0x00};
  uint8_t set_feature_report[17] = {0x00};

  int i2c_bus;
  const char *filename = "/dev/i2c-7"; // Replace with your I2C bus device
  int i2c_address = 0x4a; // Replace with the I2C address of your device
  char buf[2] = {0};

  };

  // I2c config codes

  namespace bno_msgs {

  // Configuring Reports;
  extern const uint8_t COMMAND_REQUEST;
  extern uint8_t FRS_READ_RESPONSE;
  extern uint8_t FRS_READ_REQUEST;
  extern uint8_t FRS_WRITE_RESPONSE;
  extern uint8_t FRS_WRITE_DATA;
  extern uint8_t FRS_WRITE_REQUEST;
  extern uint8_t SHTP_REPORT_ID_RESPONSE;
  extern uint8_t SHTP_REPORT_ID_REQUEST;
  extern uint8_t REBASE_TIMESTAMP;
  extern uint8_t BASE_TIMESTAMP;
  extern uint8_t GET_FEATURE_RESPONSE;
  extern uint8_t SET_FEATURE_COMMAND;
  extern uint8_t GET_FEATURE_REQUEST;
  // DCD/ ME Commands
  extern uint8_t ME_ERRORREPORT_CDE;
  extern uint8_t ME_COUNTER_CDE;
  extern uint8_t ME_TARE_CDE;
  extern uint8_t ME_INIT_CDE;
  extern uint8_t ME_SAVE_DCD_CDE;
  extern uint8_t ME_CALIBRATION_CDE;
  extern uint8_t ME_SAVE_DCD_PERIODIC_CDE;
  extern uint8_t ME_OSCILLATOR_TYPE_CDE;
  extern uint8_t ME_RESET_DCD_CDE;
  // DCD/ME Sub-commands
  extern uint8_t ME_COUNTER_GETCOUNTS_CDE;
  extern uint8_t ME_COUNTER_CLEARCOUNTS_CDE;
  extern uint8_t ME_TARE_NOW_SUBCDE;
  extern uint8_t ME_TARE_PERSIST_SUBCDE;
  extern uint8_t ME_TARE_REORIENTATION_SUBCDE;
  extern uint8_t ME_CALIBRATION_CONFIG_SUBCDE;
  extern uint8_t ME_CALIBRATION_GETCAL_SUBCDE;
  extern uint8_t ME_SAVE_DCD_PERIODIC_ENABLE_SUBCDE;
  extern uint8_t ME_SAVE_DCD_PERIODIC_DISABLE_SUBCDE;
  // BNO CONFIGURATION RECORDS;
  extern int BNO_CONF_STATIC_CALIBRATION_AGM;
  extern int BNO_CONF_NOMINAL_CALIBRATION_AGM;
  extern int BNO_CONF_STATIC_CALIBRATION_SRA;
  extern int BNO_CONF_NOMINAL_CALIBRATION_SRA;
  extern int BNO_CONF_DYNAMIC_CALIBRATION;
  extern int BNO_CONF_MOTION_ENGINE_POWER_MANAGEMENT;
  extern int BNO_CONF_SYSTEM_ORIENTATION;
  extern int BNO_CONF_PRIMARY_ACCELEROMETER_ORIENTATION;
  extern int BNO_CONF_SCREEN_ROTATION_ACCELEROMETER_ORIENTATION;
  extern int BNO_CONF_GYROSCOPE_ORIENTATION;
  extern int BNO_CONF_MAGNETOMETER_ORIENTATION;
  extern int BNO_CONF_ARVR_STABILIZATION_ROTATION_VECTOR;
  extern int BNO_CONF_ARVR_STABILIZATION_GAME_ROTATION_VECTOR;
  extern int BNO_CONF_SIGNIFICANT_MOTION_DETECTOR;
  extern int BNO_CONF_SHAKE_DETECTOR;
  extern int BNO_CONF_MAXIMUM_FUSION_PERIOD;
  extern int BNO_CONF_SERIAL_NUMBER;
  extern int BNO_CONF_ENV_SENSOR_PRESSURE_CALIBRATION;
  extern int BNO_CONF_ENV_SENSOR_TEMPERATURE_CALIBRATION;
  extern int BNO_CONF_ENV_SENSOR_HUMIDITY_CALIBRATION;
  extern int BNO_CONF_ENV_SENSOR_AMBIENT_LIGHT_CALIBRATION;
  extern int BNO_CONF_ENV_SENSOR_PROXIMITY_CALIBRATION;
  extern int BNO_CONF_ALS_CALIBRATION;
  extern int BNO_CONF_PROXIMITY_SENSOR_CALIBRATION;
  extern int BNO_CONF_PICKUP_DETECTOR;
  extern int BNO_CONF_FLIP_DETECTOR;
  extern int BNO_CONF_STABILITY_DETECTOR;
  extern int BNO_CONF_ACTIVITY_TRACKER;
  extern int BNO_CONF_SLEEP_DETECTOR;
  extern int BNO_CONF_TILT_DETECTOR;
  extern int BNO_CONF_POCKET_DETECTOR;
  extern int BNO_CONF_CIRCLE_DETECTOR;
  extern int BNO_CONF_USER_RECORD;
  extern int BNO_CONF_MOTION_ENGINE_TIME_SOURCE_SELECTION;
  extern int BNO_CONF_UART_OUTPUT_FORMAT_SELECTION;
  extern int BNO_CONF_GYRO_INTEGRATED_ROTATION_VECTOR;
  extern int BNO_CONF_FUSION_CONTROL_FLAGS;
  // Reports Summary depending on BNO device ;
  extern uint8_t BNO_REPORT_ACCELEROMETER;
  extern uint8_t BNO_REPORT_GYROSCOPE;
  extern uint8_t BNO_REPORT_MAGNETOMETER;
  extern uint8_t BNO_REPORT_LINEAR_ACCELERATION;
  extern uint8_t BNO_REPORT_ROTATION_VECTOR;
  extern uint8_t BNO_REPORT_GRAVITY;
  extern uint8_t BNO_REPORT_UNCALIBRATED_GYROSCOPE;
  extern uint8_t BNO_REPORT_GAME_ROTATION_VECTOR;
  extern uint8_t BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR;
  extern uint8_t BNO_REPORT_PRESSURE;
  extern uint8_t BNO_REPORT_AMBIENT_LIGHT;
  extern uint8_t BNO_REPORT_HUMIDITY;
  extern uint8_t BNO_REPORT_PROXIMITY;
  extern uint8_t BNO_REPORT_TEMPERATURE;
  extern uint8_t BNO_REPORT_UNCALIBRATED_MAGNETOMETER;
  extern uint8_t BNO_REPORT_TAP_DETECTOR;
  extern uint8_t BNO_REPORT_STEP_COUNTER;
  extern uint8_t BNO_REPORT_SIGNIFICANT_MOTION;
  extern uint8_t BNO_REPORT_STABILITY_CLASSIFIER;
  extern uint8_t BNO_REPORT_RAW_ACCELEROMETER;
  extern uint8_t BNO_REPORT_RAW_GYROSCOPE;
  extern uint8_t BNO_REPORT_RAW_MAGNETOMETER;
  extern uint8_t BNO_REPORT_SAR;
  extern uint8_t BNO_REPORT_STEP_DETECTOR;
  extern uint8_t BNO_REPORT_SHAKE_DETECTOR;
  extern uint8_t BNO_REPORT_FLIP_DETECTOR;
  extern uint8_t BNO_REPORT_PICKUP_DETECTOR;
  extern uint8_t BNO_REPORT_STABILITY_DETECTOR;
  extern uint8_t BNO_REPORT_ACTIVITY_CLASSIFIER;
  extern uint8_t BNO_REPORT_SLEEP_DETECTOR;
  extern uint8_t BNO_REPORT_TILT_DETECTOR;
  extern uint8_t BNO_REPORT_POCKET_DETECTOR;
  extern uint8_t BNO_REPORT_CIRCLE_DETECTOR;
  extern uint8_t BNO_REPORT_HEART_RATE_MONITOR;
  extern uint8_t BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR;
  extern uint8_t BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR;
  extern uint8_t BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR;
  // Timeouts (ms);
  extern int QUAT_READ_TIMEOUT;
  extern int PACKET_READ_TIMEOUT;
  extern int FEATURE_ENABLE_TIMEOUT;
  extern int DEFAULT_TIMEOUT;

  
  } // namespace bno_msgs
// };
}  // namespace JetBotControl




#endif // I2C_DEVICE_HPP