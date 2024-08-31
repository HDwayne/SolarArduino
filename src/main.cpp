#include "main.h"

// Initialize structs
struct STTime time;              // Struct for date and time variables
struct STLocation locationData;  // Struct for geographic locationDataation variables
struct STPosition solarPosition; // Struct for solar position variables

// Initialize time variable
DateTime lastPanelAdjustmentTime;

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
    AZIMUTH_TIME_THRESHOLD);

// Elevation Controller
ElevationController elevationController(
    ELEVATION_MOTOR_PIN_EN,
    ELEVATION_MOTOR_PWM_PIN_U,
    ELEVATION_MOTOR_PWM_PIN_D,
    AZIMUTH_DEG_MAX,
    AZIMUTH_DEG_MIN,
    ELEVATION_DEG_MAX,
    ELEVATION_DEG_MIN,
    ELEVATION_TIME_THRESHOLD,
    ELEVATION_ACTUATOR_SPEED,
    ELEVATION_ACTUATOR_LENGTH);

ezButton joyStickButton(SW_PIN);

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

  joyStickButton.setDebounceTime(500);

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

  // Calibrate the solar panel
  calibratePanel();

  // Update the solar panel position
  updatePanel();

  Serial.println(F("\n\t--- System Initialization Completed ---\n"));
}

void loop()
{
  unsigned long currentMillis = millis();
  joyStickButton.loop();

  if (joyStickButton.isPressed())
  {
    joyStick();

    resetPanelPosition();
    updatePanel();

    while (joyStickButton.isPressed())
    {
      joyStickButton.loop();
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

  azimuthController.moveFullLeft();
  elevationController.moveToMaxElevation();

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
  updateSunPos();

  elevationController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);
  azimuthController.moveToAngle(solarPosition.azimuthRefract);

  Serial.println(F("\n\t--- Solar Panel Position Updated ---\n"));
}

void joyStick()
{
  Serial.println(F("\nEntering Joystick Mode"));

  joyStickButton.loop();

  // Enable motors
  azimuthController.enableMotor();
  elevationController.enableMotor();

  // Variables to store previous command states
  uint8_t prevCommand = COMMAND_NO;

  while (!joyStickButton.isPressed())
  {
    joyStickButton.loop();

    // Read joystick values
    int xValue = analogRead(VRX_PIN);
    int yValue = analogRead(VRY_PIN);

    // Reset command
    uint8_t command = COMMAND_NO;

    // Determine left/right commands
    if (xValue < LEFT_THRESHOLD)
    {
      command |= COMMAND_LEFT;
    }
    else if (xValue > RIGHT_THRESHOLD)
    {
      command |= COMMAND_RIGHT;
    }

    // Determine up/down commands
    if (yValue < UP_THRESHOLD)
    {
      command |= COMMAND_UP;
    }
    else if (yValue > DOWN_THRESHOLD)
    {
      command |= COMMAND_DOWN;
    }

    // Process command only if there is a change
    if (command != prevCommand)
    {
      // Stop motors before changing direction to avoid conflicts
      if ((prevCommand & (COMMAND_LEFT | COMMAND_RIGHT)) != (command & (COMMAND_LEFT | COMMAND_RIGHT)))
      {
        elevationController.stopActuator();
      }
      if ((prevCommand & (COMMAND_UP | COMMAND_DOWN)) != (command & (COMMAND_UP | COMMAND_DOWN)))
      {
        azimuthController.stopMotor();
      }

      // Start motors based on new command
      if (command & COMMAND_LEFT)
      {
        elevationController.startActuatorDown();
      }
      else if (command & COMMAND_RIGHT)
      {
        elevationController.startActuatorUp();
      }

      if (command & COMMAND_UP)
      {
        azimuthController.startMotorLeft();
      }
      else if (command & COMMAND_DOWN)
      {
        azimuthController.startMotorRight();
      }

      // Update previous command
      prevCommand = command;

      // Print new command to serial if any
      String output = "";
      if (command & COMMAND_LEFT)
      {
        output += "COMMAND DOWN ";
      }
      if (command & COMMAND_RIGHT)
      {
        output += "COMMAND UP ";
      }
      if (command & COMMAND_UP)
      {
        output += "COMMAND LEFT ";
      }
      if (command & COMMAND_DOWN)
      {
        output += "COMMAND RIGHT ";
      }

      if (output != "")
      {
        Serial.println(output);
      }
    }
  }

  azimuthController.stopMotor();
  azimuthController.disableMotor();

  elevationController.stopActuator();
  elevationController.disableMotor();

  Serial.println(F("Exiting Joystick Mode"));
}

// ----------------- Sun position functions ---------------------

void updateSunPos()
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