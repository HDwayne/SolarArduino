// config.h

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// PINS
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
constexpr uint8_t AZIMUTH_MOTOR_PIN_EN = 4;     // Motor enable pin
constexpr uint8_t AZIMUTH_MOTOR_PWM_PIN_L = 6;  // Motor PWM pin (left)
constexpr uint8_t AZIMUTH_MOTOR_PWM_PIN_R = 5;  // Motor PWM pin (right)
constexpr uint8_t AZIMUTH_LIMIT_SWITCH_PIN = 7; // Limit switch pin

constexpr uint8_t ELEVATION_MOTOR_PIN_EN = 8;     // Motor enable pin
constexpr uint8_t ELEVATION_MOTOR_PWM_PIN_U = 10; // Motor PWM pin for actuator extension (up)
constexpr uint8_t ELEVATION_MOTOR_PWM_PIN_D = 9;  // Motor PWM pin for actuator retraction (down)

constexpr uint8_t JOYSTICK_VRX_PIN = A0;   // Arduino pin connected to VRX pin
constexpr uint8_t JOYSTICK_VRY_PIN = A1;   // Arduino pin connected to VRY pin
constexpr uint8_t JOYSTICK_BUTTON_PIN = 2; // Arduino pin connected to SW pin

constexpr uint8_t ANENOMETER_BUTTON_PIN = 11; // Anemometer button pin
#elif defined(ESP32)
constexpr uint8_t AZIMUTH_MOTOR_PIN_EN = GPIO_NUM_19;     // Motor enable pin
constexpr uint8_t AZIMUTH_MOTOR_PWM_PIN_L = GPIO_NUM_18;  // Motor PWM pin (left)
constexpr uint8_t AZIMUTH_MOTOR_PWM_PIN_R = GPIO_NUM_17;  // Motor PWM pin (right)
constexpr uint8_t AZIMUTH_LIMIT_SWITCH_PIN = GPIO_NUM_16; // Limit switch pin

constexpr uint8_t ELEVATION_MOTOR_PIN_EN = GPIO_NUM_26;    // Motor enable pin
constexpr uint8_t ELEVATION_MOTOR_PWM_PIN_U = GPIO_NUM_27; // Motor PWM pin for actuator extension (up)
constexpr uint8_t ELEVATION_MOTOR_PWM_PIN_D = GPIO_NUM_14; // Motor PWM pin for actuator retraction (down)

constexpr uint8_t JOYSTICK_VRX_PIN = GPIO_NUM_32;    // ESP32 pin connected to VRX pin
constexpr uint8_t JOYSTICK_VRY_PIN = GPIO_NUM_33;    // ESP32 pin connected to VRY pin
constexpr uint8_t JOYSTICK_BUTTON_PIN = GPIO_NUM_25; // ESP32 pin connected to SW pin

constexpr uint8_t ANENOMETER_BUTTON_PIN = GPIO_NUM_13; // Anemometer button pin
#else
#error "Board not supported"
#endif

// Constants
constexpr uint8_t UPDATE_PANEL_ADJUSTMENT_INTERVAL = 20; // Update panel every 15 minutes

// Solar Tracking Settings
constexpr double ST_LATITUDE = 43.8045;  // Latitude of the solar panel (in degrees)
constexpr double ST_LONGITUDE = 1.3883;  // Longitude of the solar panel (in degrees)
constexpr double ST_PRESSURE = 101.0;    // Atmospheric pressure in kPa
constexpr double ST_TEMPERATURE = 283.0; // Atmospheric temperature in Kelvin

// Solar Track Options
constexpr bool useDegrees = true;             // Input/output in degrees
constexpr bool useNorthEqualsZero = true;     // Azimuth reference: 0 = North
constexpr bool computeRefrEquatorial = false; // Do not compute refraction-corrected equatorial coordinates
constexpr bool computeDistance = false;       // Do not compute the distance to the Sun

// Azimuth Settings
constexpr uint8_t AZIMUTH_MOTOR_PWM_SPEED = 100; // Motor PWM speed

constexpr float AZIMUTH_DEG_MAX = 275.0;       // Maximum azimuth value (degrees)
constexpr float AZIMUTH_DEG_MIN = 90.0;        // Minimum azimuth value (degrees)
constexpr uint32_t AZIMUTH_TIME_THRESHOLD = 0; // Threshold in milliseconds to trigger motor adjustment (minimum rotation time)

/*
To prevent the motor from running indefinitely if the limit switch fails,
we set a maximum time before entering error mode. This time is the total
duration for a full rotation (left and right) plus a 5% margin.
Unfortunately, before calibration, the full rotation time is unknown.
Therefore, we define a maximum time before calibration to avoid the motor running indefinitely.
*/
constexpr uint32_t AZIMUTH_TIME_MAX_BEFORE_CALIBRATION = 3 * 60 * 1000;

// Elevation Settings
constexpr uint8_t ELEVATION_MOTOR_PWM_SPEED = 255; // Maximum PWM speed for the motor driver (DO NOT CHANGE else calculations will be wrong)

constexpr float ELEVATION_DEG_MAX = 90.0;        // Maximum elevation value (degrees)
constexpr float ELEVATION_DEG_MIN = 19.0;        // Minimum elevation value (degrees)
constexpr uint32_t ELEVATION_TIME_THRESHOLD = 0; // Threshold in milliseconds to trigger motor adjustment (minimum rotation time)

constexpr float ELEVATION_ACTUATOR_SPEED = 5.0;    // Actuator speed in mm/s
constexpr float ELEVATION_ACTUATOR_LENGTH = 350.0; // Actuator length in mm

// uncomment the following line to force the full travel time in seconds
#define FORCE_TIME_FULL_TRAVEL 95 // If you observe that the actuator does not reach the end positions (real and predicted can differ), you can force the full travel time in seconds

// Joystick Settings
constexpr uint32_t JOYSTICK_BUTTON_DEBOUNCE = 5000; // Debounce time in milliseconds
constexpr uint16_t JOYSTICK_THRESHOLD = 250;        // Joystick threshold
// Anemometer Settings
constexpr unsigned long ANENOMETER_SAFE_DURATION = 15 * 60 * 1000; // Anemometer safe duration in milliseconds (15 (minutes) * 60 (seconds) * 1000 (milliseconds))

// MQTT Settings
constexpr char MQTT_SERVER[] = "replace_with_your_mqtt_server";
constexpr uint16_t MQTT_PORT = 1883;           // MQTT server port
constexpr char MQTT_USER[] = "";               // MQTT username
constexpr char MQTT_PASSWORD[] = "";           // MQTT password
constexpr char DEVICE_NAME[] = "SolarTracker"; // Device name

// NTP Settings

constexpr char NTP_SERVER1[] = "europe.pool.ntp.org";
constexpr char NTP_SERVER2[] = "time.google.com";
constexpr char NTP_SERVER3[] = "time.aws.com";

#endif // CONFIG_H