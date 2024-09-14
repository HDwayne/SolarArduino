#include "main.h"

// Initialize structs
struct STTime currentTime;       // Struct for date and time variables
struct STLocation locationData;  // Struct for geographic locationDataation variables
struct STPosition solarPosition; // Struct for solar position variables

// Initialize time variable
DateTime lastPanelAdjustmentTime;

// RTC Module
RTC_DS1307 rtc;

unsigned long lastPanelAdjustmentMillis = 0;
const unsigned long panelAdjustmentIntervalMillis = 60000; // check every minute

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

  // Initialize Modules
#if defined(MODULE_WIFI_H)
  wifiModule.init();
#endif // MODULE_WIFI_H

  // Initialize the RTC module
  initRTC();

  // initialize joystick
  initJoystick();

  // Set initial location data for solar calculations
  locationData.latitude = ST_LATITUDE;
  locationData.longitude = ST_LONGITUDE;
  locationData.pressure = ST_PRESSURE;
  locationData.temperature = ST_TEMPERATURE;

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
      ; // avoid multiple calls
  }

  if (currentMillis - lastPanelAdjustmentMillis >= panelAdjustmentIntervalMillis)
  {
    lastPanelAdjustmentMillis = currentMillis;

    DateTime now = rtc.now();
    uint32_t secc = now.unixtime() - lastPanelAdjustmentTime.unixtime();
    uint16_t minutesDiff = secc / 60;

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

void initRTC()
{
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

  Serial.print(F("[INFO] RTC is running"));
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

  currentTime.year = lastPanelAdjustmentTime.year();
  currentTime.month = lastPanelAdjustmentTime.month();
  currentTime.day = lastPanelAdjustmentTime.day();
  currentTime.hour = lastPanelAdjustmentTime.hour() - TIMEZONE;
  currentTime.minute = lastPanelAdjustmentTime.minute();
  currentTime.second = lastPanelAdjustmentTime.second();

  SolTrack(currentTime, locationData, &solarPosition, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

  Serial.print(F("Date: "));
  Serial.print(currentTime.year);
  Serial.print(F("-"));
  if (currentTime.month < 10)
    Serial.print(F("0")); // Zero padding for single digit months
  Serial.print(currentTime.month);
  Serial.print(F("-"));
  if (currentTime.day < 10)
    Serial.print(F("0")); // Zero padding for single digit days
  Serial.println(currentTime.day);

  Serial.print(F("Time: "));
  if (currentTime.hour + TIMEZONE < 10)
    Serial.print(F("0")); // Zero padding for single digit hours
  Serial.print(currentTime.hour + TIMEZONE);
  Serial.print(F(":"));
  if (currentTime.minute < 10)
    Serial.print(F("0")); // Zero padding for single digit minutes
  Serial.print(currentTime.minute);
  Serial.print(F(":"));
  if (currentTime.second < 10)
    Serial.print(F("0")); // Zero padding for single digit seconds
  Serial.println(currentTime.second);

  Serial.println();

  Serial.print(F("Corrected Azimuth: "));
  Serial.print(solarPosition.azimuthRefract, 2);
  Serial.print(F("°\tAltitude: "));
  Serial.print(solarPosition.altitudeRefract, 2);
  Serial.println(F("°"));

  elevationController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);
  azimuthController.moveToAngle(solarPosition.azimuthRefract);

  Serial.println(F("\n\t--- Solar Panel Position Updated ---\n"));
}

// ----------------- Joystick control functions ---------------------

void initJoystick()
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
