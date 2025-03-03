#include "ConfigModule.h"

#include "utils/Logger.h"

extern Logger Log;

// Constructor
ConfigModule::ConfigModule()
{
}

// Initialize the module
void ConfigModule::begin()
{
  preferences.begin("config", false);
  if (!loadConfig())
  {
    // First time or failed to load, set defaults and save
    setDefaultConfig();
    saveConfig();
  }
}

// Load configuration from NVS
bool ConfigModule::loadConfig()
{
  size_t expectedSize = sizeof(ConfigData);
  size_t actualSize = preferences.getBytesLength("configData");
  if (actualSize == expectedSize)
  {
    preferences.getBytes("configData", &configData, expectedSize);
    return true;
  }
  return false;
}

// Save configuration to NVS
void ConfigModule::saveConfig()
{
  preferences.putBytes("configData", &configData, sizeof(ConfigData));
}

// Get reference to configuration data
ConfigData &ConfigModule::getConfig()
{
  return configData;
}

// Reset configuration to default values
void ConfigModule::resetConfig()
{
  setDefaultConfig();
  saveConfig();
}

void ConfigModule::setDefaultConfig()
{
  resetPinsConfig();
  resetConstantsConfig();
  resetSolarTrackingConfig();
  resetSolTrackOptionsConfig();
  resetAzimuthConfig();
  resetElevationConfig();
  resetJoystickConfig();
  resetAnemometerConfig();
  resetMQTTConfig();
  resetNTPConfig();
  resetWiFiConfig();
}

void ConfigModule::resetPinsConfig()
{
  configData.AZIMUTH_MOTOR_PIN_EN = GPIO_NUM_19;
  configData.AZIMUTH_MOTOR_PWM_PIN_L = GPIO_NUM_18;
  configData.AZIMUTH_MOTOR_PWM_PIN_R = GPIO_NUM_17;
  configData.AZIMUTH_LIMIT_SWITCH_PIN = GPIO_NUM_16;

  configData.ELEVATION_MOTOR_PIN_EN = GPIO_NUM_26;
  configData.ELEVATION_MOTOR_PWM_PIN_U = GPIO_NUM_27;
  configData.ELEVATION_MOTOR_PWM_PIN_D = GPIO_NUM_14;

  configData.JOYSTICK_VRX_PIN = GPIO_NUM_32;
  configData.JOYSTICK_VRY_PIN = GPIO_NUM_33;
  configData.JOYSTICK_BUTTON_PIN = GPIO_NUM_25;

  configData.ANENOMETER_BUTTON_PIN = GPIO_NUM_13;
}

void ConfigModule::resetConstantsConfig()
{
  configData.UPDATE_PANEL_ADJUSTMENT_INTERVAL = 20;
}

void ConfigModule::resetSolarTrackingConfig()
{
  configData.ST_LATITUDE = 1.1234;
  configData.ST_LONGITUDE = 1.1234;
  configData.ST_PRESSURE = 101.0;
  configData.ST_TEMPERATURE = 283.0;
}

void ConfigModule::resetSolTrackOptionsConfig()
{
  configData.useDegrees = true;
  configData.useNorthEqualsZero = true;
  configData.computeRefrEquatorial = false;
  configData.computeDistance = false;
}

void ConfigModule::resetAzimuthConfig()
{
  configData.AZIMUTH_MOTOR_PWM_SPEED = 100;
  configData.AZIMUTH_DEG_MAX = 275.0;
  configData.AZIMUTH_DEG_MIN = 90.0;
  configData.AZIMUTH_TIME_THRESHOLD = 0;
  configData.AZIMUTH_TIME_MAX_BEFORE_CALIBRATION = 180000;
}

void ConfigModule::resetElevationConfig()
{
  configData.ELEVATION_MOTOR_PWM_SPEED = 255;
  configData.ELEVATION_DEG_MAX = 90.0;
  configData.ELEVATION_DEG_MIN = 19.0;
  configData.ELEVATION_TIME_THRESHOLD = 0;
  configData.ELEVATION_ACTUATOR_SPEED = 5.0;
  configData.ELEVATION_ACTUATOR_LENGTH = 350.0;
  configData.FORCE_TIME_FULL_TRAVEL = 0;
}

void ConfigModule::resetJoystickConfig()
{
  configData.JOYSTICK_BUTTON_DEBOUNCE = 5000;
  configData.JOYSTICK_THRESHOLD = 250;
}

void ConfigModule::resetAnemometerConfig()
{
  configData.ANENOMETER_SAFE_DURATION = 900000;
}

void ConfigModule::resetMQTTConfig()
{
  // MQTT Settings
  strncpy(configData.MQTT_SERVER, "", sizeof(configData.MQTT_SERVER) - 1);
  configData.MQTT_SERVER[sizeof(configData.MQTT_SERVER) - 1] = '\0';
  configData.MQTT_PORT = 1883;
  strncpy(configData.MQTT_USER, "", sizeof(configData.MQTT_USER));
  configData.MQTT_USER[sizeof(configData.MQTT_USER) - 1] = '\0';
  strncpy(configData.MQTT_PASSWORD, "", sizeof(configData.MQTT_PASSWORD));
  configData.MQTT_PASSWORD[sizeof(configData.MQTT_PASSWORD) - 1] = '\0';
  strncpy(configData.DEVICE_NAME, "SolarTracker", sizeof(configData.DEVICE_NAME) - 1);
  configData.DEVICE_NAME[sizeof(configData.DEVICE_NAME) - 1] = '\0';
}

void ConfigModule::resetNTPConfig()
{
  // NTP Settings
  strncpy(configData.NTP_SERVER1, "europe.pool.ntp.org", sizeof(configData.NTP_SERVER1) - 1);
  configData.NTP_SERVER1[sizeof(configData.NTP_SERVER1) - 1] = '\0';
  strncpy(configData.NTP_SERVER2, "time.google.com", sizeof(configData.NTP_SERVER2) - 1);
  configData.NTP_SERVER2[sizeof(configData.NTP_SERVER2) - 1] = '\0';
  strncpy(configData.NTP_SERVER3, "time.aws.com", sizeof(configData.NTP_SERVER3) - 1);
  configData.NTP_SERVER3[sizeof(configData.NTP_SERVER3) - 1] = '\0';
}

void ConfigModule::resetWiFiConfig()
{
  // Wi-Fi Credentials
  strncpy(configData.WIFI_SSID, "", sizeof(configData.WIFI_SSID));
  configData.WIFI_SSID[sizeof(configData.WIFI_SSID) - 1] = '\0';
  strncpy(configData.WIFI_PASSWORD, "", sizeof(configData.WIFI_PASSWORD));
  configData.WIFI_PASSWORD[sizeof(configData.WIFI_PASSWORD) - 1] = '\0';
}

// ------------------------ SETTERS ----------------------------------

void ConfigModule::setAzimuthMotorPinEn(uint8_t value)
{
  configData.AZIMUTH_MOTOR_PIN_EN = value;
}

void ConfigModule::setAzimuthMotorPwmPinL(uint8_t value)
{
  configData.AZIMUTH_MOTOR_PWM_PIN_L = value;
}

void ConfigModule::setAzimuthMotorPwmPinR(uint8_t value)
{
  configData.AZIMUTH_MOTOR_PWM_PIN_R = value;
}

void ConfigModule::setAzimuthLimitSwitchPin(uint8_t value)
{
  configData.AZIMUTH_LIMIT_SWITCH_PIN = value;
}

void ConfigModule::setElevationMotorPinEn(uint8_t value)
{
  configData.ELEVATION_MOTOR_PIN_EN = value;
}

void ConfigModule::setElevationMotorPwmPinU(uint8_t value)
{
  configData.ELEVATION_MOTOR_PWM_PIN_U = value;
}

void ConfigModule::setElevationMotorPwmPinD(uint8_t value)
{
  configData.ELEVATION_MOTOR_PWM_PIN_D = value;
}

void ConfigModule::setJoystickVrxPin(uint8_t value)
{
  configData.JOYSTICK_VRX_PIN = value;
}

void ConfigModule::setJoystickVryPin(uint8_t value)
{
  configData.JOYSTICK_VRY_PIN = value;
}

void ConfigModule::setJoystickButtonPin(uint8_t value)
{
  configData.JOYSTICK_BUTTON_PIN = value;
}

void ConfigModule::setAnenometerButtonPin(uint8_t value)
{
  configData.ANENOMETER_BUTTON_PIN = value;
}

void ConfigModule::setUpdatePanelAdjustmentInterval(uint8_t value)
{
  configData.UPDATE_PANEL_ADJUSTMENT_INTERVAL = value;
}

void ConfigModule::setSTLatitude(double value)
{
  configData.ST_LATITUDE = value;
}

void ConfigModule::setSTLongitude(double value)
{
  configData.ST_LONGITUDE = value;
}

void ConfigModule::setSTPressure(double value)
{
  configData.ST_PRESSURE = value;
}

void ConfigModule::setSTTemperature(double value)
{
  configData.ST_TEMPERATURE = value;
}

void ConfigModule::setUseDegrees(bool value)
{
  configData.useDegrees = value;
}

void ConfigModule::setUseNorthEqualsZero(bool value)
{
  configData.useNorthEqualsZero = value;
}

void ConfigModule::setComputeRefrEquatorial(bool value)
{
  configData.computeRefrEquatorial = value;
}

void ConfigModule::setComputeDistance(bool value)
{
  configData.computeDistance = value;
}

void ConfigModule::setAzimuthMotorPWMSpeed(uint8_t value)
{
  configData.AZIMUTH_MOTOR_PWM_SPEED = value;
}

void ConfigModule::setAzimuthDegMax(float value)
{
  configData.AZIMUTH_DEG_MAX = value;
}

void ConfigModule::setAzimuthDegMin(float value)
{
  configData.AZIMUTH_DEG_MIN = value;
}

void ConfigModule::setAzimuthTimeThreshold(uint32_t value)
{
  configData.AZIMUTH_TIME_THRESHOLD = value;
}

void ConfigModule::setAzimuthTimeMaxBeforeCalibration(uint32_t value)
{
  configData.AZIMUTH_TIME_MAX_BEFORE_CALIBRATION = value;
}

void ConfigModule::setElevationMotorPWMSpeed(uint8_t value)
{
  configData.ELEVATION_MOTOR_PWM_SPEED = value;
}

void ConfigModule::setElevationDegMax(float value)
{
  configData.ELEVATION_DEG_MAX = value;
}

void ConfigModule::setElevationDegMin(float value)
{
  configData.ELEVATION_DEG_MIN = value;
}

void ConfigModule::setElevationTimeThreshold(uint32_t value)
{
  configData.ELEVATION_TIME_THRESHOLD = value;
}

void ConfigModule::setElevationActuatorSpeed(float value)
{
  configData.ELEVATION_ACTUATOR_SPEED = value;
}

void ConfigModule::setElevationActuatorLength(float value)
{
  configData.ELEVATION_ACTUATOR_LENGTH = value;
}

void ConfigModule::setForceTimeFullTravel(uint32_t value)
{
  configData.FORCE_TIME_FULL_TRAVEL = value;
}

void ConfigModule::setJoystickButtonDebounce(uint32_t value)
{
  configData.JOYSTICK_BUTTON_DEBOUNCE = value;
}

void ConfigModule::setJoystickThreshold(uint16_t value)
{
  configData.JOYSTICK_THRESHOLD = value;
}

void ConfigModule::setAnenometerSafeDuration(unsigned long value)
{
  configData.ANENOMETER_SAFE_DURATION = value;
}

void ConfigModule::setMQTTServer(const char *value)
{
  strncpy(configData.MQTT_SERVER, value, sizeof(configData.MQTT_SERVER));
}

void ConfigModule::setMQTTPort(uint16_t value)
{
  configData.MQTT_PORT = value;
}

void ConfigModule::setMQTTUser(const char *value)
{
  strncpy(configData.MQTT_USER, value, sizeof(configData.MQTT_USER));
}

void ConfigModule::setMQTTPassword(const char *value)
{
  strncpy(configData.MQTT_PASSWORD, value, sizeof(configData.MQTT_PASSWORD));
}

void ConfigModule::setDeviceName(const char *value)
{
  strncpy(configData.DEVICE_NAME, value, sizeof(configData.DEVICE_NAME));
}

void ConfigModule::setNTPServer1(const char *value)
{
  strncpy(configData.NTP_SERVER1, value, sizeof(configData.NTP_SERVER1));
}

void ConfigModule::setNTPServer2(const char *value)
{
  strncpy(configData.NTP_SERVER2, value, sizeof(configData.NTP_SERVER2));
}

void ConfigModule::setNTPServer3(const char *value)
{
  strncpy(configData.NTP_SERVER3, value, sizeof(configData.NTP_SERVER3));
}

void ConfigModule::setWIFISSID(const char *value)
{
  strncpy(configData.WIFI_SSID, value, sizeof(configData.WIFI_SSID));
}

void ConfigModule::setWIFIPassword(const char *value)
{
  strncpy(configData.WIFI_PASSWORD, value, sizeof(configData.WIFI_PASSWORD));
}

void ConfigModule::printConfig()
{
  Log.println("Configuration:");
  Log.printf("AZIMUTH_MOTOR_PIN_EN: %d\n", configData.AZIMUTH_MOTOR_PIN_EN);
  Log.printf("AZIMUTH_MOTOR_PWM_PIN_L: %d\n", configData.AZIMUTH_MOTOR_PWM_PIN_L);
  Log.printf("AZIMUTH_MOTOR_PWM_PIN_R: %d\n", configData.AZIMUTH_MOTOR_PWM_PIN_R);
  Log.printf("AZIMUTH_LIMIT_SWITCH_PIN: %d\n", configData.AZIMUTH_LIMIT_SWITCH_PIN);
  Log.printf("ELEVATION_MOTOR_PIN_EN: %d\n", configData.ELEVATION_MOTOR_PIN_EN);
  Log.printf("ELEVATION_MOTOR_PWM_PIN_U: %d\n", configData.ELEVATION_MOTOR_PWM_PIN_U);
  Log.printf("ELEVATION_MOTOR_PWM_PIN_D: %d\n", configData.ELEVATION_MOTOR_PWM_PIN_D);
  Log.printf("JOYSTICK_VRX_PIN: %d\n", configData.JOYSTICK_VRX_PIN);
  Log.printf("JOYSTICK_VRY_PIN: %d\n", configData.JOYSTICK_VRY_PIN);
  Log.printf("JOYSTICK_BUTTON_PIN: %d\n", configData.JOYSTICK_BUTTON_PIN);
  Log.printf("ANENOMETER_BUTTON_PIN: %d\n", configData.ANENOMETER_BUTTON_PIN);
  Log.printf("UPDATE_PANEL_ADJUSTMENT_INTERVAL: %d\n", configData.UPDATE_PANEL_ADJUSTMENT_INTERVAL);
  Log.printf("ST_LATITUDE: %f\n", configData.ST_LATITUDE);
  Log.printf("ST_LONGITUDE: %f\n", configData.ST_LONGITUDE);
  Log.printf("ST_PRESSURE: %f\n", configData.ST_PRESSURE);
  Log.printf("ST_TEMPERATURE: %f\n", configData.ST_TEMPERATURE);
  Log.printf("useDegrees: %d\n", configData.useDegrees);
  Log.printf("useNorthEqualsZero: %d\n", configData.useNorthEqualsZero);
  Log.printf("computeRefrEquatorial: %d\n", configData.computeRefrEquatorial);
  Log.printf("computeDistance: %d\n", configData.computeDistance);
  Log.printf("AZIMUTH_MOTOR_PWM_SPEED: %d\n", configData.AZIMUTH_MOTOR_PWM_SPEED);
  Log.printf("AZIMUTH_DEG_MAX: %f\n", configData.AZIMUTH_DEG_MAX);
  Log.printf("AZIMUTH_DEG_MIN: %f\n", configData.AZIMUTH_DEG_MIN);
  Log.printf("AZIMUTH_TIME_THRESHOLD: %d\n", configData.AZIMUTH_TIME_THRESHOLD);
  Log.printf("AZIMUTH_TIME_MAX_BEFORE_CALIBRATION: %d\n", configData.AZIMUTH_TIME_MAX_BEFORE_CALIBRATION);
  Log.printf("ELEVATION_MOTOR_PWM_SPEED: %d\n", configData.ELEVATION_MOTOR_PWM_SPEED);
  Log.printf("ELEVATION_DEG_MAX: %f\n", configData.ELEVATION_DEG_MAX);
  Log.printf("ELEVATION_DEG_MIN: %f\n", configData.ELEVATION_DEG_MIN);
  Log.printf("ELEVATION_TIME_THRESHOLD: %d\n", configData.ELEVATION_TIME_THRESHOLD);
  Log.printf("ELEVATION_ACTUATOR_SPEED: %f\n", configData.ELEVATION_ACTUATOR_SPEED);
  Log.printf("ELEVATION_ACTUATOR_LENGTH: %f\n", configData.ELEVATION_ACTUATOR_LENGTH);
  Log.printf("FORCE_TIME_FULL_TRAVEL: %d\n", configData.FORCE_TIME_FULL_TRAVEL);
  Log.printf("JOYSTICK_BUTTON_DEBOUNCE: %d\n", configData.JOYSTICK_BUTTON_DEBOUNCE);
  Log.printf("JOYSTICK_THRESHOLD: %d\n", configData.JOYSTICK_THRESHOLD);
  Log.printf("ANENOMETER_SAFE_DURATION: %lu\n", configData.ANENOMETER_SAFE_DURATION);
  Log.printf("MQTT_SERVER: %s\n", configData.MQTT_SERVER);
  Log.printf("MQTT_PORT: %d\n", configData.MQTT_PORT);
  Log.printf("MQTT_USER: %s\n", configData.MQTT_USER);
  Log.printf("MQTT_PASSWORD: %s\n", configData.MQTT_PASSWORD);
  Log.printf("DEVICE_NAME: %s\n", configData.DEVICE_NAME);
  Log.printf("NTP_SERVER1: %s\n", configData.NTP_SERVER1);
  Log.printf("NTP_SERVER2: %s\n", configData.NTP_SERVER2);
  Log.printf("NTP_SERVER3: %s\n", configData.NTP_SERVER3);
  Log.printf("WIFI_SSID: %s\n", configData.WIFI_SSID);
  Log.printf("WIFI_PASSWORD: %s\n", configData.WIFI_PASSWORD);
}