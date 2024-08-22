#include <Arduino.h>
#include <RTClib.h>
#include <SPI.h>
#include "stdlib.h"
#include "SolTrack.h"
#include "AzimuthController.h"
#include "ElevationController.h"

// ----------------- Constants and Variables -----------------

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
struct STTime time;              // Struct for date and time variables
struct STLocation locationData;  // Struct for geographic locationDataation variables
struct STPosition solarPosition; // Struct for solar position variables

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
DateTime lastPanelAdjustmentTime; // Last time the solar panel was adjusted

// RTC Module
RTC_DS1307 rtc;

// Azimuth Controller
AzimuthController azimuthController(
    AZIMUTH_MOTOR_PIN_EN,
    AZIMUTH_MOTOR_PWM_PIN_L,
    AZIMUTH_MOTOR_PWM_PIN_R,
    AZIMUTH_MOTOR_PWM_SPEED,
    AZIMUTH_LIMIT_SWITCH_PIN,
    AZIMUTH_DEG_MAX,
    AZIMUTH_DEG_MIN,
    AZIMUTH_DEG_THRESHOLD,
    AZIMUTH_TIME_THRESHOLD);

// Elevation Controller
ElevationController elevationController(
    ELEVATION_MOTOR_PIN_EN,
    ELEVATION_MOTOR_PWM_PIN_U,
    ELEVATION_MOTOR_PWM_PIN_D,
    ELEVATION_DEG_MAX,
    ELEVATION_DEG_MIN,
    ELEVATION_TIME_THRESHOLD,
    ELEVATION_ACTUATOR_SPEED,
    ELEVATION_ACTUATOR_LENGTH);

// ----------------- Function Prototypes -----------------

void UpdateSunPos();
void printSunPos();
void printDateTime(DateTime now);

// ----------------- Setup and Loop -----------------

void setup()
{
  delay(1000); // stabilization time

  // Initialize Serial
  Serial.begin(9600);
  while (!Serial)
    ;

  // Initialize the system components
  Serial.println(F("\n\t--- System Initialization ---\n"));

  // Initialize the RTC module
  if (!rtc.begin())
  {
    Serial.println(F("[ERROR] RTC initialization failed!"));
    while (1)
      ; // Halt if RTC initialization fails
  }

  if (!rtc.isrunning())
  {
    Serial.println(F("[ERROR] RTC is not running."));
    // Set the RTC time to the compile time
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    while (1)
      ; // Halt if RTC is not running
  }
  else
  {
    lastPanelAdjustmentTime = rtc.now();
    Serial.print(F("[INFO] RTC is running with the following time: "));
    printDateTime(lastPanelAdjustmentTime);
  }

  // Set initial location data for solar calculations
  locationData.latitude = ST_LATITUDE;
  locationData.longitude = ST_LONGITUDE;
  locationData.pressure = ST_PRESSURE;
  locationData.temperature = ST_TEMPERATURE;

  // Perform the azimuth calibration procedure
  azimuthController.calibrate();
  delay(3000);

  // Perform the elevation calibration procedure
  elevationController.calibrate();
  delay(3000);

  // Move the solar panel for the first time
  Serial.println(F("\n\t--- First Azimuth Adjustment ---\n"));
  UpdateSunPos();
  azimuthController.moveToAngle(solarPosition.azimuthRefract);
  delay(3000);
  elevationController.moveToAngle(solarPosition.altitudeRefract);

  Serial.println(F("\n\t--- System Ready, Entering Main Loop ---\n"));
}

void loop()
{
  DateTime now = rtc.now();

  if (now.hour() != lastPanelAdjustmentTime.hour())
  {
    Serial.print(F("\n\t--- New Hour Detected ---\n"));
    lastPanelAdjustmentTime = now;

    UpdateSunPos();
    azimuthController.moveToAngle(solarPosition.azimuthRefract);
    delay(3000);
    elevationController.moveToAngle(solarPosition.altitudeRefract);
  }
  else
  {
    Serial.print(F("."));
  }

  delay(60000);
}

// ----------------- RTC functions -----------------------------

void printDateTime(DateTime now)
{
  // Print date in YYYY-MM-DD format
  Serial.print(now.year(), DEC);
  Serial.print('-');
  if (now.month() < 10)
    Serial.print('0');
  Serial.print(now.month(), DEC);
  Serial.print('-');
  if (now.day() < 10)
    Serial.print('0');
  Serial.print(now.day(), DEC);

  Serial.print(' ');

  // Print time in HH:MM:SS format
  if (now.hour() < 10)
    Serial.print('0');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if (now.minute() < 10)
    Serial.print('0');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if (now.second() < 10)
    Serial.print('0');
  Serial.println(now.second(), DEC);
}

// ----------------- Solar tracking functions ------------------

void UpdateSunPos()
{
  time.year = lastPanelAdjustmentTime.year();
  time.month = lastPanelAdjustmentTime.month();
  time.day = lastPanelAdjustmentTime.day();
  time.hour = lastPanelAdjustmentTime.hour() - TIMEZONE;
  time.minute = lastPanelAdjustmentTime.minute();
  time.second = lastPanelAdjustmentTime.second();

  SolTrack(time, locationData, &solarPosition, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

  printSunPos();
}

void printSunPos()
{
  Serial.println(F("\n\t--- Sun Position Data ---\n"));

  Serial.print(F("Date: "));
  Serial.print(time.year);
  Serial.print(F("-"));
  if (time.month < 10)
    Serial.print(F("0")); // Zero padding for single digit months
  Serial.print(time.month);
  Serial.print(F("-"));
  if (time.day < 10)
    Serial.print(F("0")); // Zero padding for single digit days
  Serial.println(time.day);

  Serial.print(F("Time: "));
  if (time.hour + TIMEZONE < 10)
    Serial.print(F("0")); // Zero padding for single digit hours
  Serial.print(time.hour + TIMEZONE);
  Serial.print(F(":"));
  if (time.minute < 10)
    Serial.print(F("0")); // Zero padding for single digit minutes
  Serial.print(time.minute);
  Serial.print(F(":"));
  if (time.second < 10)
    Serial.print(F("0")); // Zero padding for single digit seconds
  Serial.println(time.second);

  Serial.println();

  Serial.print(F("Corrected Azimuth: "));
  Serial.print(solarPosition.azimuthRefract, 2);
  Serial.print(F("°\tAltitude: "));
  Serial.print(solarPosition.altitudeRefract, 2);
  Serial.println(F("°"));

  Serial.println(F("\n\t--- End of Sun Position Data ---\n"));
}