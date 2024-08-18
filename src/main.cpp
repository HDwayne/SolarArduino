#include <Arduino.h>
#include <ezButton.h>
#include <BTS7960.h>
#include <RTClib.h>
#include <SPI.h>
#include "stdlib.h"
#include "SolTrack.h"

// Pin Definitions
#define MOTOR_PIN_EN 4
#define MOTOR_PWM_PIN_L 6
#define MOTOR_PWM_PIN_R 5
#define LIMIT_SWITCH_PIN 7

// Motor Settings
#define MOTOR_PWM_SPEED 60

// Solar Tracking Settings
#define ST_LATITUDE 43.8045  // Latitude of the solar panel (in degrees)
#define ST_LONGITUDE 1.3883  // Longitude of the solar panel (in degrees)
#define ST_PRESSURE 101.0    // Atmospheric pressure in kPa
#define ST_TEMPERATURE 283.0 // Atmospheric temperature in Kelvin

// Timezone
#define TIMEZONE 2 // UTC+1 in winter, UTC+2 in summer

// Structs for SolTrack
struct STTime time;              // Struct for date and time variables
struct STLocation locationData;  // Struct for geographic locationDataation variables
struct STPosition solarPosition; // Struct for solar position variables

// Motor Controller and Limit Switch
BTS7960 motorController(MOTOR_PIN_EN, MOTOR_PWM_PIN_L, MOTOR_PWM_PIN_R);
ezButton limitSwitch(LIMIT_SWITCH_PIN);

// RTC Module
RTC_DS1307 rtc;

// Solar Track Options
const int useDegrees = 1;            // Input/output in degrees
const int useNorthEqualsZero = 1;    // Azimuth reference: 0 = North
const int computeRefrEquatorial = 0; // Do not compute refraction-corrected equatorial coordinates
const int computeDistance = 0;       // Do not compute the distance to the Sun

// Variables
unsigned long fullRotationDuration; // Duration of a full rotation in milliseconds
float currentAzimuth = 0.0;         // Current azimuth position of the solar panel
float degreesPerMs = 0.0;           // Degrees the motor turns per millisecond

#define AZIMUTH_MAX 310.0              // Maximum azimuth value
#define AZIMUTH_MIN 50.0               // Minimum azimuth value
#define AZIMUTH_DEG_THRESHOLD 10.0     // Threshold in degrees to trigger motor adjustment (minimum rotation angle)
#define AZIMUTH_TIME_THRESHOLD 10000.0 // Threshold in milliseconds to trigger motor adjustment (minimum rotation time)

DateTime lastAzimuthUpdateTime;

// Function Prototypes
void azimuthCalibrationProcedure();
void azimuthFullLeft();
void azimuthFullRight();
void startMotorLeft();
void startMotorRight();
void stopMotor();
void waitForButtonPress();
void initializeSystem();
void UpdateSunPos();
void printAllData();
void printDateTime(DateTime now);
void adjustAzimuth();
bool waitForButtonPressOrDelay(unsigned long delayTime);

// ----------------- Setup and Loop -----------------

void setup()
{
  delay(1000); // stabilization time

  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial)
    ;

  // Initialize the system components
  initializeSystem();
  delay(1000);

  // Perform the azimuth calibration procedure
  azimuthCalibrationProcedure();
  delay(1000);

  // Perform the first azimuth adjustment
  Serial.println(F("\n\t--- First Azimuth Adjustment ---\n"));
  UpdateSunPos();
  adjustAzimuth();
  Serial.println(F("\n\t--- System Ready, Entering Main Loop ---\n"));
}

void loop()
{
  DateTime now = rtc.now();

  if (now.hour() != lastAzimuthUpdateTime.hour())
  {
    Serial.print(F("\n\t--- New Hour Detected ---\n"));
    lastAzimuthUpdateTime = now;

    UpdateSunPos();
    adjustAzimuth();
  }
  else
  {
    Serial.print(F("."));
  }

  delay(60000);
}

// ----------------- System initialization functions -----------

void initializeSystem()
{
  Serial.println(F("\n\t--- System Initialization ---\n"));

  // Initialize the limit switch with debounce time
  limitSwitch.setDebounceTime(100);

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
    lastAzimuthUpdateTime = rtc.now();
    Serial.print(F("[INFO] RTC is running with the following time: "));
    printDateTime(lastAzimuthUpdateTime);
  }

  // Set initial location data for solar calculations
  locationData.latitude = ST_LATITUDE;
  locationData.longitude = ST_LONGITUDE;
  locationData.pressure = ST_PRESSURE;
  locationData.temperature = ST_TEMPERATURE;

  Serial.println(F("\n\t--- System initialized. ---\n"));
}

// ----------------- Azimuth control functions -----------------

void azimuthCalibrationProcedure()
{
  Serial.println(F("\n\t--- Starting Calibration Procedure ---\n"));

  azimuthFullRight();
  delay(3000);

  unsigned long pressStartTime = millis();
  azimuthFullLeft();

  fullRotationDuration = millis() - pressStartTime;
  Serial.print(F("\n\tCalibration Duration: "));
  Serial.print(fullRotationDuration);
  Serial.println(F(" ms"));

  // Calculate degrees per millisecond
  degreesPerMs = 360.0 / fullRotationDuration;
  Serial.print(F("\tDegrees per millisecond: "));
  Serial.print(degreesPerMs, 6);
  Serial.println(F("°/ms\n"));

  Serial.println(F("\n\t--- Calibration procedure completed. ---\n"));
}

void azimuthFullLeft()
{
  Serial.println(F("-> Moving to full left position"));
  motorController.Enable();

  startMotorLeft();
  waitForButtonPress();
  stopMotor();

  motorController.Disable();

  currentAzimuth = 0.0;
  Serial.println(F("-> Full left position reached"));
}

void azimuthFullRight()
{
  Serial.println(F("-> Moving to full right position"));
  motorController.Enable();

  startMotorRight();
  waitForButtonPress();
  stopMotor();

  motorController.Disable();

  currentAzimuth = 360.0;
  Serial.println(F("-> Full right position reached"));
}

void adjustAzimuth()
{
  Serial.println(F("\n\t--- Adjusting Azimuth ---\n"));
  float solarAzimuth = solarPosition.azimuthRefract; // Get the computed solar azimuth

  if (solarAzimuth < AZIMUTH_MIN || solarAzimuth > AZIMUTH_MAX)
  {
    Serial.println(F("[INFO] Solar azimuth is out of the tracking range. Cannot adjust azimuth."));

    if (currentAzimuth != 0.0)
    {
      Serial.println(F("[INFO] Moving to the initial position."));
      azimuthFullLeft();
    }

    return;
  }

  // Check if the difference between the current panel azimuth and the solar azimuth exceeds the threshold
  float azimuthDifference = fabs(solarAzimuth - currentAzimuth);
  if (azimuthDifference < AZIMUTH_DEG_THRESHOLD)
  {
    Serial.println(F("[INFO] Azimuth is already aligned according to the threshold."));
    return;
  }

  // Calculate the time needed to move the panel to the solar azimuth
  float timeToMove = azimuthDifference / degreesPerMs; // Time in milliseconds
  if (timeToMove < AZIMUTH_TIME_THRESHOLD)
  {
    Serial.println(F("[INFO] Time to move is less than the minimal required time. Cannot adjust azimuth."));
    return;
  }

  Serial.print(F("[INFO] Azimuth difference: "));
  Serial.print(azimuthDifference, 2);
  Serial.print(F("°. Time to move: "));
  Serial.print(timeToMove);
  Serial.println(F(" ms"));

  motorController.Enable();

  // Determine the direction of movement and move the motor
  if (solarAzimuth > currentAzimuth)
  {
    Serial.println(F("[ADJUST] Moving motor to the right to align with the sun."));
    currentAzimuth += azimuthDifference;
    startMotorRight();
    if (waitForButtonPressOrDelay(timeToMove))
    {
      Serial.println(F("[WARNING] Limit switch was pressed during adjustment."));
      currentAzimuth = 360.0;
    }
  }
  else
  {
    Serial.println(F("[ADJUST] Moving motor to the left to align with the sun."));
    currentAzimuth -= azimuthDifference;
    startMotorLeft();
    if (waitForButtonPressOrDelay(timeToMove))
    {
      Serial.println(F("[WARNING] Limit switch was pressed during adjustment."));
      currentAzimuth = 0.0;
    }
  }

  // Stop the motor after adjustment
  stopMotor();
  motorController.Disable();

  Serial.print(F("[ADJUST] Azimuth aligned. Current azimuth: "));
  Serial.println(currentAzimuth, 2);

  Serial.println(F("\n\t--- End of Azimuth Adjustment ---\n"));
}

// ----------------- Motor control functions -------------------

void startMotorLeft()
{
  Serial.println(F("[MOTOR] Starting motor. Direction: left."));
  motorController.TurnLeft(MOTOR_PWM_SPEED);
}

void startMotorRight()
{
  Serial.println(F("[MOTOR] Starting motor. Direction: right."));
  motorController.TurnRight(MOTOR_PWM_SPEED);
}

void stopMotor()
{
  Serial.println(F("[MOTOR] Stopping motor."));
  motorController.Stop();
}

// ----------------- Button control functions ------------------

// function blocks until the button is pressed
void waitForButtonPress()
{
  while (true)
  {
    limitSwitch.loop();
    if (limitSwitch.isPressed())
    {
      Serial.println(F("[SWITCH] Limit switch pressed."));
      break;
    }
  }
  limitSwitch.loop();
}

// function blocks until the button is pressed or the delay time has passed. Returns true if the button was pressed.
bool waitForButtonPressOrDelay(unsigned long delayTime)
{
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime)
  {
    limitSwitch.loop();
    if (limitSwitch.isPressed())
    {
      Serial.println(F("[SWITCH] Limit switch pressed."));
      return true;
    }
  }
  limitSwitch.loop();
  return false;
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
  time.year = lastAzimuthUpdateTime.year();
  time.month = lastAzimuthUpdateTime.month();
  time.day = lastAzimuthUpdateTime.day();
  time.hour = lastAzimuthUpdateTime.hour() - TIMEZONE;
  time.minute = lastAzimuthUpdateTime.minute();
  time.second = lastAzimuthUpdateTime.second();

  SolTrack(time, locationData, &solarPosition, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

  printAllData();
}

void printAllData()
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