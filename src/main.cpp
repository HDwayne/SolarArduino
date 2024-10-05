#include "main.h"

// Initialize structs
struct STTime currentTime;       // Struct for date and time variables
struct STLocation locationData;  // Struct for geographic locationDataation variables
struct STPosition solarPosition; // Struct for solar position variables

// Initialize time variable
DateTime lastPanelAdjustmentTime;

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

  rtcModule.init();
  if (rtcModule.lostPower())
  {
    // TODO GET from NTP
    rtcModule.adjustFromLocal(__DATE__, __TIME__);
  }

  // Initialize Modules
#if defined(MODULE_WIFI_H)
  wifiModule.init();
#endif // MODULE_WIFI_H

#if defined(MODULE_MQTT_H)
  mqttModule.init();
#endif // MODULE_MQTT_H

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

#if defined(MODULE_MQTT_H)
  mqttModule.loop();
#endif // MODULE_MQTT_H

  if (anenometerModule.isTriggered())
  {
    AnenometerMode();

    updatePanel();

    while (anenometerModule.isTriggered())
      ; // avoid multiple calls
  }

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

    DateTime now = rtcModule.getDateTimeUTC();
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

// ----------------- Panel control functions ---------------------

void resetPanelPosition()
{
  Serial.println(F("\n\t--- Resetting Solar Panel Position ---\n"));

  elevationController.moveToMaxElevation();
  if (azimuthController.moveFullLeft() < 0)
  {
    Serial.println(F("[ERROR] An error occurred during azimuth adjustment. "));
    errorMode();
  }

  Serial.println(F("\n\t--- Solar Panel Position Reset ---\n"));
}

void calibratePanel()
{
  Serial.println(F("\n\t--- Starting Solar Panel Calibration ---\n"));

  elevationController.calibrate();
  if (azimuthController.calibrate() < 0)
  {
    Serial.println(F("[ERROR] An error occurred during azimuth calibration. "));
    errorMode();
  }

  Serial.println(F("\n\t--- Solar Panel Calibration Completed ---\n"));
}

void updatePanel()
{
  Serial.println(F("\n\t--- Updating Solar Panel Position ---\n"));
  lastPanelAdjustmentTime = rtcModule.getDateTimeUTC();
  DateTime lastPanelAdjustmentTimeLocal = rtcModule.getDateTimeLocal();

  currentTime.year = lastPanelAdjustmentTime.year();
  currentTime.month = lastPanelAdjustmentTime.month();
  currentTime.day = lastPanelAdjustmentTime.day();
  currentTime.hour = lastPanelAdjustmentTime.hour();
  currentTime.minute = lastPanelAdjustmentTime.minute();
  currentTime.second = lastPanelAdjustmentTime.second();

  SolTrack(currentTime, locationData, &solarPosition, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

  Serial.println();

  rtcModule.printDateTime(lastPanelAdjustmentTime);

  Serial.print(F("Corrected Azimuth: "));
  Serial.print(solarPosition.azimuthRefract, 2);
  Serial.print(F("°\tAltitude: "));
  Serial.print(solarPosition.altitudeRefract, 2);
  Serial.println(F("°"));

  float newelevation = elevationController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);
  float newazimuth = azimuthController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);

  if (newazimuth < 0)
  {
    Serial.println(F("[ERROR] An error occurred during azimuth adjustment. "));
    errorMode();
  }

#if defined(MODULE_MQTT_H)
  mqttModule.updateField(SOLAR_AZIMUTH, &solarPosition.azimuthRefract);
  mqttModule.updateField(SOLAR_ELEVATION, &solarPosition.altitudeRefract);
  mqttModule.updateField(PANEL_AZIMUTH, &newazimuth);
  mqttModule.updateField(PANEL_ELEVATION, &newelevation);
  mqttModule.updateField(LAST_PANEL_ADJUSTMENT_TIME, &lastPanelAdjustmentTimeLocal);
  DateTime nextPanelAdjustmentTimeLocal = lastPanelAdjustmentTimeLocal.unixtime() + UPDATE_PANEL_ADJUSTMENT_INTERVAL * 60;
  mqttModule.updateField(NEXT_PANEL_ADJUSTMENT_TIME, &nextPanelAdjustmentTimeLocal);
#endif // MODULE_MQTT_H

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

// ----------------- Anenometer control functions ---------------------

void AnenometerMode()
{
  Serial.println(F("\nEntering Anenometer Mode"));

  elevationController.moveToMaxElevation();

  unsigned long countdownStart = millis();
  unsigned long countdownEnd = countdownStart + ANENOMETER_SAFE_DURATION;

  while (millis() < countdownEnd)
  {
    if (anenometerModule.isTriggered())
    {
      Serial.println(F("Anenometer triggered, resetting countdown"));

      countdownStart = millis();
      countdownEnd = countdownStart + ANENOMETER_SAFE_DURATION;
    }
    delay(100);
  }

  Serial.println(F("Exiting Anenometer Mode"));
}

// ----------------- ESP32 Error mode ---------------------

void errorMode()
{
  Serial.println(F("\n\t--- Entering Error Mode ---\n"));
  elevationController.moveToMaxElevation();

#if defined(ESP32)
  Serial.println(F("Entering Deep Sleep Mode"));
  esp_deep_sleep_start();
#endif // ESP32

  Serial.println(F("\n\tEntering infinite loop"));
  while (true)
    delay(1000);
}