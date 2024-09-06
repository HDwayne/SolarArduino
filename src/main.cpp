#include "main.h"

// Initialize structs
struct STTime time;              // Struct for date and time variables
struct STLocation locationData;  // Struct for geographic locationDataation variables
struct STPosition solarPosition; // Struct for solar position variables

// Initialize time variable
DateTime lastPanelAdjustmentTime;

// RTC Module
RTC_DS1307 rtc;

unsigned long lastPanelAdjustmentMillis = 0;
unsigned long panelAdjustmentIntervalMillis = 60000; // check every minute

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
    Serial.print(F("[INFO] RTC is running with the following time: "));
    printDateTime(rtc.now());
  }

  // Set initial location data for solar calculations
  locationData.latitude = ST_LATITUDE;
  locationData.longitude = ST_LONGITUDE;
  locationData.pressure = ST_PRESSURE;
  locationData.temperature = ST_TEMPERATURE;

  // initialize joystick
  initializeJoystick();

  // Calibrate the solar panel
  calibratePanel();

  // Update the solar panel position
  updatePanel();

  Serial.println(F("\n\t--- System Initialization Completed ---\n"));
}

void loop()
{
  unsigned long currentMillis = millis();

  if (joystickController.isPressed())
  {
    JoystickMode();

    resetPanelPosition();
    updatePanel();

    while (joystickController.isPressed())
    {
      // avoid bouncing
    }
  }

  if (currentMillis - lastPanelAdjustmentMillis >= panelAdjustmentIntervalMillis)
  {
    lastPanelAdjustmentMillis = currentMillis;

    DateTime now = rtc.now();
    int nowTotalMinutes = now.hour() * 60 + now.minute();
    int lastAdjustmentTotalMinutes = lastPanelAdjustmentTime.hour() * 60 + lastPanelAdjustmentTime.minute();
    int minutesDiff = nowTotalMinutes - lastAdjustmentTotalMinutes;

    if (minutesDiff < 0)
    {
      minutesDiff += 1440; // 24 hours in minutes
    }

    if (minutesDiff >= UPDATE_PANEL_ADJUSTMENT_INTERVAL)
    {
      Serial.print(F("\n\t--- New Solar Panel Adjustment ---\n"));
      updatePanel();
    }
    else
    {
      Serial.print(F("."));
    }
  }
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

// ----------------- Panel control functions ---------------------

void resetPanelPosition()
{
  Serial.println(F("\n\t--- Resetting Solar Panel Position ---\n"));

  elevationController.moveToMaxElevation();
  azimuthController.moveFullLeft();

  Serial.println(F("\n\t--- Solar Panel Position Reset ---\n"));
}

void calibratePanel()
{
  Serial.println(F("\n\t--- Starting Solar Panel Calibration ---\n"));

  elevationController.calibrate();
  azimuthController.calibrate();

  Serial.println(F("\n\t--- Solar Panel Calibration Completed ---\n"));
}

void updatePanel()
{
  Serial.println(F("\n\t--- Updating Solar Panel Position ---\n"));
  lastPanelAdjustmentTime = rtc.now();

  time.year = lastPanelAdjustmentTime.year();
  time.month = lastPanelAdjustmentTime.month();
  time.day = lastPanelAdjustmentTime.day();
  time.hour = lastPanelAdjustmentTime.hour() - TIMEZONE;
  time.minute = lastPanelAdjustmentTime.minute();
  time.second = lastPanelAdjustmentTime.second();

  SolTrack(time, locationData, &solarPosition, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

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

  elevationController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);
  azimuthController.moveToAngle(solarPosition.azimuthRefract);

  Serial.println(F("\n\t--- Solar Panel Position Updated ---\n"));
}

// ----------------- Joystick control functions ---------------------

void initializeJoystick()
{
  joystickController.setOnDown([]()
                               { azimuthController.startMotorRight(); });

  joystickController.setOnUp([]()
                             { azimuthController.startMotorLeft(); });

  joystickController.setOnLeaveLR([]()
                                  { elevationController.stopActuator(); });

  joystickController.setOnLeaveUD([]()
                                  { azimuthController.stopMotor(); });

  joystickController.setOnLeft([]()
                               { elevationController.startActuatorDown(); });

  joystickController.setOnRight([]()
                                { elevationController.startActuatorUp(); });
}

void JoystickMode()
{
  Serial.println(F("\nEntering Joystick Mode"));

  azimuthController.enableMotor();
  elevationController.enableMotor();

  joystickController.clearCommand();

  while (!joystickController.isPressed())
  {
    joystickController.executeCommand();
  }

  azimuthController.stopMotor();
  azimuthController.disableMotor();

  elevationController.stopActuator();
  elevationController.disableMotor();

  Serial.println(F("Exiting Joystick Mode"));
}
