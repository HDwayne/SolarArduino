#ifndef CONFIG_MODULE_H
#define CONFIG_MODULE_H

#include <Arduino.h>
#include "utils/Logger.h"

struct ConfigData
{
  // PINS
  uint8_t AZIMUTH_MOTOR_PIN_EN;
  uint8_t AZIMUTH_MOTOR_PWM_PIN_L;
  uint8_t AZIMUTH_MOTOR_PWM_PIN_R;
  uint8_t AZIMUTH_LIMIT_SWITCH_PIN;

  uint8_t ELEVATION_MOTOR_PIN_EN;
  uint8_t ELEVATION_MOTOR_PWM_PIN_U;
  uint8_t ELEVATION_MOTOR_PWM_PIN_D;

  uint8_t JOYSTICK_VRX_PIN;
  uint8_t JOYSTICK_VRY_PIN;
  uint8_t JOYSTICK_BUTTON_PIN;

  uint8_t ANENOMETER_BUTTON_PIN;

  // Constants
  uint8_t UPDATE_PANEL_ADJUSTMENT_INTERVAL;

  // Solar Tracking Settings
  double ST_LATITUDE;    // Latitude of the solar panel (in degrees)
  double ST_LONGITUDE;   // Longitude of the solar panel (in degrees)
  double ST_PRESSURE;    // Atmospheric pressure in kPa
  double ST_TEMPERATURE; // Atmospheric temperature in Kelvin

  // Solar Track Options
  bool useDegrees;            // Input/output in degrees
  bool useNorthEqualsZero;    // Azimuth reference: 0 = North
  bool computeRefrEquatorial; // Do not compute refraction-corrected equatorial coordinates
  bool computeDistance;       // Do not compute the distance to the Sun

  // Azimuth Settings
  uint8_t AZIMUTH_MOTOR_PWM_SPEED;              // Motor PWM speed
  float AZIMUTH_DEG_MAX;                        // Maximum azimuth value (degrees)
  float AZIMUTH_DEG_MIN;                        // Minimum azimuth value (degrees)
  uint32_t AZIMUTH_TIME_THRESHOLD;              // Threshold in milliseconds to trigger motor adjustment
  uint32_t AZIMUTH_TIME_MAX_BEFORE_CALIBRATION; // Max time before entering error mode

  // Elevation Settings
  uint8_t ELEVATION_MOTOR_PWM_SPEED; // Maximum PWM speed for the motor driver
  float ELEVATION_DEG_MAX;           // Maximum elevation value (degrees)
  float ELEVATION_DEG_MIN;           // Minimum elevation value (degrees)
  uint32_t ELEVATION_TIME_THRESHOLD; // Threshold in milliseconds to trigger motor adjustment
  float ELEVATION_ACTUATOR_SPEED;    // Actuator speed in mm/s
  float ELEVATION_ACTUATOR_LENGTH;   // Actuator length in mm
  uint32_t FORCE_TIME_FULL_TRAVEL;   // Force full travel time in seconds

  // Joystick Settings
  uint32_t JOYSTICK_BUTTON_DEBOUNCE; // Debounce time in milliseconds
  uint16_t JOYSTICK_THRESHOLD;       // Joystick threshold

  // Anemometer Settings
  unsigned long ANENOMETER_SAFE_DURATION; // Safe duration in milliseconds

  // MQTT Settings
  char MQTT_SERVER[64];
  uint16_t MQTT_PORT;
  char MQTT_USER[32];
  char MQTT_PASSWORD[32];
  char DEVICE_NAME[32];

  // NTP Settings
  char NTP_SERVER1[64];
  char NTP_SERVER2[64];
  char NTP_SERVER3[64];

  // WIFI Credentials
  char WIFI_SSID[32];
  char WIFI_PASSWORD[32];
};

#if defined(ESP32)

#include <Preferences.h>

class ConfigModule
{
public:
  ConfigModule();
  void begin();
  bool loadConfig();
  void saveConfig();
  ConfigData &getConfig();

  void resetConfig();
  void resetPinsConfig();
  void resetConstantsConfig();
  void resetSolarTrackingConfig();
  void resetSolTrackOptionsConfig();
  void resetAzimuthConfig();
  void resetElevationConfig();
  void resetJoystickConfig();
  void resetAnemometerConfig();
  void resetMQTTConfig();
  void resetNTPConfig();
  void resetWiFiConfig();

  void printConfig();

  // Setters
  void setUpdatePanelAdjustmentInterval(uint8_t value);

  void setAzimuthMotorPinEn(uint8_t value);
  void setAzimuthMotorPwmPinL(uint8_t value);
  void setAzimuthMotorPwmPinR(uint8_t value);
  void setAzimuthLimitSwitchPin(uint8_t value);
  void setElevationMotorPinEn(uint8_t value);
  void setElevationMotorPwmPinU(uint8_t value);
  void setElevationMotorPwmPinD(uint8_t value);
  void setJoystickVrxPin(uint8_t value);
  void setJoystickVryPin(uint8_t value);
  void setJoystickButtonPin(uint8_t value);
  void setAnenometerButtonPin(uint8_t value);

  void setSTLatitude(double value);
  void setSTLongitude(double value);
  void setSTPressure(double value);
  void setSTTemperature(double value);

  void setUseDegrees(bool value);
  void setUseNorthEqualsZero(bool value);
  void setComputeRefrEquatorial(bool value);
  void setComputeDistance(bool value);

  void setAzimuthMotorPWMSpeed(uint8_t value);
  void setAzimuthDegMax(float value);
  void setAzimuthDegMin(float value);
  void setAzimuthTimeThreshold(uint32_t value);
  void setAzimuthTimeMaxBeforeCalibration(uint32_t value);

  void setElevationMotorPWMSpeed(uint8_t value);
  void setElevationDegMax(float value);
  void setElevationDegMin(float value);
  void setElevationTimeThreshold(uint32_t value);
  void setElevationActuatorSpeed(float value);
  void setElevationActuatorLength(float value);
  void setForceTimeFullTravel(uint32_t value);

  void setJoystickButtonDebounce(uint32_t value);
  void setJoystickThreshold(uint16_t value);

  void setAnenometerSafeDuration(unsigned long value);

  void setMQTTServer(const char *value);
  void setMQTTPort(uint16_t value);
  void setMQTTUser(const char *value);
  void setMQTTPassword(const char *value);
  void setDeviceName(const char *value);

  void setNTPServer1(const char *value);
  void setNTPServer2(const char *value);
  void setNTPServer3(const char *value);

  void setWIFISSID(const char *value);
  void setWIFIPassword(const char *value);

  // Getters
  uint8_t getAzimuthMotorPinEn() { return configData.AZIMUTH_MOTOR_PIN_EN; }
  uint8_t getAzimuthMotorPwmPinL() { return configData.AZIMUTH_MOTOR_PWM_PIN_L; }
  uint8_t getAzimuthMotorPwmPinR() { return configData.AZIMUTH_MOTOR_PWM_PIN_R; }
  uint8_t getAzimuthLimitSwitchPin() { return configData.AZIMUTH_LIMIT_SWITCH_PIN; }

  uint8_t getElevationMotorPinEn() { return configData.ELEVATION_MOTOR_PIN_EN; }
  uint8_t getElevationMotorPwmPinU() { return configData.ELEVATION_MOTOR_PWM_PIN_U; }
  uint8_t getElevationMotorPwmPinD() { return configData.ELEVATION_MOTOR_PWM_PIN_D; }

  uint8_t getJoystickVrxPin() { return configData.JOYSTICK_VRX_PIN; }
  uint8_t getJoystickVryPin() { return configData.JOYSTICK_VRY_PIN; }
  uint8_t getJoystickButtonPin() { return configData.JOYSTICK_BUTTON_PIN; }

  uint8_t getAnenometerButtonPin() { return configData.ANENOMETER_BUTTON_PIN; }

  uint8_t getUpdatePanelAdjustmentInterval() { return configData.UPDATE_PANEL_ADJUSTMENT_INTERVAL; }

  double getSTLatitude() { return configData.ST_LATITUDE; }
  double getSTLongitude() { return configData.ST_LONGITUDE; }
  double getSTPressure() { return configData.ST_PRESSURE; }
  double getSTTemperature() { return configData.ST_TEMPERATURE; }

  bool getUseDegrees() { return configData.useDegrees; }
  bool getUseNorthEqualsZero() { return configData.useNorthEqualsZero; }
  bool getComputeRefrEquatorial() { return configData.computeRefrEquatorial; }
  bool getComputeDistance() { return configData.computeDistance; }

  uint8_t getAzimuthMotorPWMSpeed() { return configData.AZIMUTH_MOTOR_PWM_SPEED; }
  float getAzimuthDegMax() { return configData.AZIMUTH_DEG_MAX; }
  float getAzimuthDegMin() { return configData.AZIMUTH_DEG_MIN; }
  uint32_t getAzimuthTimeThreshold() { return configData.AZIMUTH_TIME_THRESHOLD; }
  uint32_t getAzimuthTimeMaxBeforeCalibration() { return configData.AZIMUTH_TIME_MAX_BEFORE_CALIBRATION; }

  uint8_t getElevationMotorPWMSpeed() { return configData.ELEVATION_MOTOR_PWM_SPEED; }
  float getElevationDegMax() { return configData.ELEVATION_DEG_MAX; }
  float getElevationDegMin() { return configData.ELEVATION_DEG_MIN; }
  uint32_t getElevationTimeThreshold() { return configData.ELEVATION_TIME_THRESHOLD; }
  float getElevationActuatorSpeed() { return configData.ELEVATION_ACTUATOR_SPEED; }
  float getElevationActuatorLength() { return configData.ELEVATION_ACTUATOR_LENGTH; }
  uint32_t getForceTimeFullTravel() { return configData.FORCE_TIME_FULL_TRAVEL; }

  uint32_t getJoystickButtonDebounce() { return configData.JOYSTICK_BUTTON_DEBOUNCE; }
  uint16_t getJoystickThreshold() { return configData.JOYSTICK_THRESHOLD; }

  unsigned long getAnenometerSafeDuration() { return configData.ANENOMETER_SAFE_DURATION; }

  const char *getMQTTServer() { return configData.MQTT_SERVER; }
  uint16_t getMQTTPort() { return configData.MQTT_PORT; }
  const char *getMQTTUser() { return configData.MQTT_USER; }
  const char *getMQTTPassword() { return configData.MQTT_PASSWORD; }
  const char *getDeviceName() { return configData.DEVICE_NAME; }

  const char *getNTPServer1() { return configData.NTP_SERVER1; }
  const char *getNTPServer2() { return configData.NTP_SERVER2; }
  const char *getNTPServer3() { return configData.NTP_SERVER3; }

  const char *getWIFISSID() { return configData.WIFI_SSID; }
  const char *getWIFIPassword() { return configData.WIFI_PASSWORD; }

private:
  void setDefaultConfig();
  ConfigData configData;
  Preferences preferences;
};

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)

// For Arduino boards, configurations are static and read-only
class ConfigModule
{
public:
  ConfigModule();
  void begin();                        // Empty function for compatibility
  const ConfigData &getConfig() const; // Returns a const reference to the config data
  void printConfig();                  // Print all configuration values

  // Getters
  uint8_t getAzimuthMotorPinEn() { return configData.AZIMUTH_MOTOR_PIN_EN; }
  uint8_t getAzimuthMotorPwmPinL() { return configData.AZIMUTH_MOTOR_PWM_PIN_L; }
  uint8_t getAzimuthMotorPwmPinR() { return configData.AZIMUTH_MOTOR_PWM_PIN_R; }
  uint8_t getAzimuthLimitSwitchPin() { return configData.AZIMUTH_LIMIT_SWITCH_PIN; }

  uint8_t getElevationMotorPinEn() { return configData.ELEVATION_MOTOR_PIN_EN; }
  uint8_t getElevationMotorPwmPinU() { return configData.ELEVATION_MOTOR_PWM_PIN_U; }
  uint8_t getElevationMotorPwmPinD() { return configData.ELEVATION_MOTOR_PWM_PIN_D; }

  uint8_t getJoystickVrxPin() { return configData.JOYSTICK_VRX_PIN; }
  uint8_t getJoystickVryPin() { return configData.JOYSTICK_VRY_PIN; }
  uint8_t getJoystickButtonPin() { return configData.JOYSTICK_BUTTON_PIN; }

  uint8_t getAnenometerButtonPin() { return configData.ANENOMETER_BUTTON_PIN; }

  uint8_t getUpdatePanelAdjustmentInterval() { return configData.UPDATE_PANEL_ADJUSTMENT_INTERVAL; }

  double getSTLatitude() { return configData.ST_LATITUDE; }
  double getSTLongitude() { return configData.ST_LONGITUDE; }
  double getSTPressure() { return configData.ST_PRESSURE; }
  double getSTTemperature() { return configData.ST_TEMPERATURE; }

  bool getUseDegrees() { return configData.useDegrees; }
  bool getUseNorthEqualsZero() { return configData.useNorthEqualsZero; }
  bool getComputeRefrEquatorial() { return configData.computeRefrEquatorial; }
  bool getComputeDistance() { return configData.computeDistance; }

  uint8_t getAzimuthMotorPWMSpeed() { return configData.AZIMUTH_MOTOR_PWM_SPEED; }
  float getAzimuthDegMax() { return configData.AZIMUTH_DEG_MAX; }
  float getAzimuthDegMin() { return configData.AZIMUTH_DEG_MIN; }
  uint32_t getAzimuthTimeThreshold() { return configData.AZIMUTH_TIME_THRESHOLD; }
  uint32_t getAzimuthTimeMaxBeforeCalibration() { return configData.AZIMUTH_TIME_MAX_BEFORE_CALIBRATION; }

  uint8_t getElevationMotorPWMSpeed() { return configData.ELEVATION_MOTOR_PWM_SPEED; }
  float getElevationDegMax() { return configData.ELEVATION_DEG_MAX; }
  float getElevationDegMin() { return configData.ELEVATION_DEG_MIN; }
  uint32_t getElevationTimeThreshold() { return configData.ELEVATION_TIME_THRESHOLD; }
  float getElevationActuatorSpeed() { return configData.ELEVATION_ACTUATOR_SPEED; }
  float getElevationActuatorLength() { return configData.ELEVATION_ACTUATOR_LENGTH; }
  uint32_t getForceTimeFullTravel() { return configData.FORCE_TIME_FULL_TRAVEL; }

  uint32_t getJoystickButtonDebounce() { return configData.JOYSTICK_BUTTON_DEBOUNCE; }
  uint16_t getJoystickThreshold() { return configData.JOYSTICK_THRESHOLD; }

  unsigned long getAnenometerSafeDuration() { return configData.ANENOMETER_SAFE_DURATION; }

private:
  static const ConfigData configData; // Static constant configuration data
};

#else
#error "Board not supported"
#endif

extern ConfigModule configModule;

#endif // CONFIG_MODULE_H
