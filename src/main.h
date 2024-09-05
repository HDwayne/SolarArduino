// SolarTracker.h

#ifndef SOLAR_TRACKER_H
#define SOLAR_TRACKER_H

#include <Arduino.h>
#include <RTClib.h>
#include <SPI.h>
#include "stdlib.h"
#include "SolTrack.h"
#include "controllers/AzimuthController.h"
#include "controllers/ElevationController.h"
#include "ezButton.h"

// Constants
constexpr int UPDATE_PANEL_ADJUSTMENT_INTERVAL = 20; // Update panel every 15 minutes

// Timezone
constexpr int TIMEZONE = 2; // FR: UTC+1 in winter, UTC+2 in summer

// Solar Tracking Settings
constexpr double ST_LATITUDE = 43.8045;  // Latitude of the solar panel (in degrees)
constexpr double ST_LONGITUDE = 1.3883;  // Longitude of the solar panel (in degrees)
constexpr double ST_PRESSURE = 101.0;    // Atmospheric pressure in kPa
constexpr double ST_TEMPERATURE = 283.0; // Atmospheric temperature in Kelvin

// Solar Track Options
constexpr int useDegrees = 1;            // Input/output in degrees
constexpr int useNorthEqualsZero = 1;    // Azimuth reference: 0 = North
constexpr int computeRefrEquatorial = 0; // Do not compute refraction-corrected equatorial coordinates
constexpr int computeDistance = 0;       // Do not compute the distance to the Sun

// Azimuth Settings
constexpr int AZIMUTH_MOTOR_PIN_EN = 4;      // Motor enable pin
constexpr int AZIMUTH_MOTOR_PWM_PIN_L = 6;   // Motor PWM pin (left)
constexpr int AZIMUTH_MOTOR_PWM_PIN_R = 5;   // Motor PWM pin (right)
constexpr int AZIMUTH_MOTOR_PWM_SPEED = 100; // Motor PWM speed
constexpr int AZIMUTH_LIMIT_SWITCH_PIN = 7;  // Limit switch pin

constexpr float AZIMUTH_DEG_MAX = 275.0;            // Maximum azimuth value (degrees)
constexpr float AZIMUTH_DEG_MIN = 90.0;             // Minimum azimuth value (degrees)
constexpr unsigned long AZIMUTH_TIME_THRESHOLD = 0; // Threshold in milliseconds to trigger motor adjustment (minimum rotation time)

// Elevation Settings
constexpr int ELEVATION_MOTOR_PIN_EN = 8;     // Motor enable pin
constexpr int ELEVATION_MOTOR_PWM_PIN_U = 10; // Motor PWM pin for actuator extension (up)
constexpr int ELEVATION_MOTOR_PWM_PIN_D = 9;  // Motor PWM pin for actuator retraction (down)

constexpr float ELEVATION_DEG_MAX = 90.0;             // Maximum elevation value (degrees)
constexpr float ELEVATION_DEG_MIN = 19.0;             // Minimum elevation value (degrees)
constexpr unsigned long ELEVATION_TIME_THRESHOLD = 0; // Threshold in milliseconds to trigger motor adjustment (minimum rotation time)

constexpr float ELEVATION_ACTUATOR_SPEED = 5.0;    // Actuator speed in mm/s
constexpr float ELEVATION_ACTUATOR_LENGTH = 350.0; // Actuator length in mm

// Joystick Settings
constexpr int VRX_PIN = A0; // Arduino pin connected to VRX pin
constexpr int VRY_PIN = A1; // Arduino pin connected to VRY pin
constexpr int SW_PIN = 2;   // Arduino pin connected to SW pin

constexpr int LEFT_THRESHOLD = 400;
constexpr int RIGHT_THRESHOLD = 800;
constexpr int UP_THRESHOLD = 400;
constexpr int DOWN_THRESHOLD = 800;

constexpr uint8_t COMMAND_NO = 0x00;
constexpr uint8_t COMMAND_LEFT = 0x01;
constexpr uint8_t COMMAND_RIGHT = 0x02;
constexpr uint8_t COMMAND_UP = 0x04;
constexpr uint8_t COMMAND_DOWN = 0x08;

// Structs for SolTrack
extern struct STTime time;              // Struct for date and time variables
extern struct STLocation locationData;  // Struct for geographic locationDataation variables
extern struct STPosition solarPosition; // Struct for solar position variables

// Time variables
extern DateTime lastPanelAdjustmentTime; // Last time the solar panel was adjusted

// RTC Module
extern RTC_DS1307 rtc;

// Azimuth Controller
extern AzimuthController azimuthController;

// Elevation Controller
extern ElevationController elevationController;

// Azimuth Controller
extern AzimuthController azimuthController;

// Elevation Controller
extern ElevationController elevationController;

// Function Prototypes
void resetPanelPosition();
void calibratePanel();
void updatePanel();
void joyStick();
void updateSunPos();
void printSunPos();
void printDateTime(DateTime now);

#endif // SOLAR_TRACKER_H
