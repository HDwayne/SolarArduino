// SolarTracker.h

#ifndef SOLAR_TRACKER_H
#define SOLAR_TRACKER_H

#include <Arduino.h>
#include <RTClib.h>
#include <SPI.h>
#include "stdlib.h"
#include "SolTrack.h"
#include "AzimuthController.h"
#include "ElevationController.h"

// Timezone
#define TIMEZONE 2 // FR: UTC+1 in winter, UTC+2 in summer

// Solar Tracking Settings
#define ST_LATITUDE 43.8045  // Latitude of the solar panel (in degrees)
#define ST_LONGITUDE 1.3883  // Longitude of the solar panel (in degrees)
#define ST_PRESSURE 101.0    // Atmospheric pressure in kPa
#define ST_TEMPERATURE 283.0 // Atmospheric temperature in Kelvin

// Solar Track Options
const int useDegrees = 1;            // Input/output in degrees
const int useNorthEqualsZero = 1;    // Azimuth reference: 0 = North
const int computeRefrEquatorial = 0; // Do not compute refraction-corrected equatorial coordinates
const int computeDistance = 0;       // Do not compute the distance to the Sun

// Structs for SolTrack
extern struct STTime time;              // Struct for date and time variables
extern struct STLocation locationData;  // Struct for geographic locationDataation variables
extern struct STPosition solarPosition; // Struct for solar position variables

// Azimuth Settings
#define AZIMUTH_MOTOR_PIN_EN 4     // Motor enable pin
#define AZIMUTH_MOTOR_PWM_PIN_L 6  // Motor PWM pin (left)
#define AZIMUTH_MOTOR_PWM_PIN_R 5  // Motor PWM pin (right)
#define AZIMUTH_MOTOR_PWM_SPEED 60 // Motor PWM speed
#define AZIMUTH_LIMIT_SWITCH_PIN 7 // Limit switch pin

#define AZIMUTH_DEG_MAX 310.0         // Maximum azimuth value (degrees)
#define AZIMUTH_DEG_MIN 50.0          // Minimum azimuth value (degrees)
#define AZIMUTH_DEG_THRESHOLD 10.0    // Threshold in degrees to trigger motor adjustment (minimum rotation angle)
#define AZIMUTH_TIME_THRESHOLD 6000.0 // Threshold in milliseconds to trigger motor adjustment (minimum rotation time)

// Elevation Settings
#define ELEVATION_MOTOR_PIN_EN 8     // Motor enable pin
#define ELEVATION_MOTOR_PWM_PIN_U 10 // Motor PWM pin for actuator extension (up)
#define ELEVATION_MOTOR_PWM_PIN_D 9  // Motor PWM pin for actuator retraction (down)

#define ELEVATION_DEG_MAX 90.0          // Maximum elevation value (degrees)
#define ELEVATION_DEG_MIN 15.0          // Minimum elevation value (degrees)
#define ELEVATION_TIME_THRESHOLD 2000.0 // Threshold in milliseconds to trigger motor adjustment (minimum rotation time)

#define ELEVATION_ACTUATOR_SPEED 5.0    // Actuator speed in mm/s
#define ELEVATION_ACTUATOR_LENGTH 350.0 // Actuator length in mm

// Time variables
extern DateTime lastPanelAdjustmentTime; // Last time the solar panel was adjusted

// RTC Module
extern RTC_DS1307 rtc;

// Azimuth Controller
extern AzimuthController azimuthController;

// Elevation Controller
extern ElevationController elevationController;

// Function Prototypes
void UpdateSunPos();
void printSunPos();
void printDateTime(DateTime now);

#endif // SOLAR_TRACKER_H
