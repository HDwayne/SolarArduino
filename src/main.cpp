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
  delay(1000);

  configModule.begin();

  delay(1000);

#if defined(MODULE_WIFI_H)
  wifiModule.init();
#endif // MODULE_WIFI_H

  delay(1000);

  // Initialize Serial
  Log.begin(9600);

  configModule.printConfig();

  // Initialize the system components
  Log.println(F("\n\t--- System Initialization ---\n"));

#if defined(MODULE_ANENOMETER_H)
  anenometerModule.init();
#endif // MODULE_ANENOMETER_H

  rtcModule.init();
  if (rtcModule.lostPower())
  {
#if defined(ESP32)
    rtcModule.adjustFromNTP();
#else
    Log.println(F("[ERROR] RTC module lost power and no NTP server available. Please set the date and time manually."));
    errorMode();
    // rtcModule.adjustFromLocal(__DATE__, __TIME__);
#endif
  }

#if defined(MODULE_MQTT_H)
  mqttModule.init();
#endif // MODULE_MQTT_H

#if defined(MODULE_OTA_H)
  otaModule.init();
#endif // MODULE_OTA_H

#if defined(MODULE_WEBSERVER_H)
  webServerModule.begin();
#endif // MODULE_WEBSERVER_H

#if defined(JOYSTICK_CONTROLLER_H)
  initJoystick();
#endif // JOYSTICK_CONTROLLER_H

  // Set initial location data for solar calculations
  locationData.latitude = configModule.getSTLatitude();
  locationData.longitude = configModule.getSTLongitude();
  locationData.pressure = configModule.getSTPressure();
  locationData.temperature = configModule.getSTTemperature();

  // Calibrate the solar panel
  elevationController.init();
  if (azimuthController.init() < 0)
  {
    Log.println(F("[ERROR] An error occurred during azimuth calibration. "));
    errorMode();
  }

  // Update the solar panel position
  updatePanel();

  Log.println(F("\n\t--- System Initialization Completed ---\n"));
}

void loop()
{
  unsigned long currentMillis = millis();

#if defined(MODULE_MQTT_H)
  mqttModule.loop();
#endif // MODULE_MQTT_H

#if defined(MODULE_OTA_H)
  otaModule.loop();
#endif // MODULE_OTA_H

#if defined(MODULE_WEBSERVER_H)
  if (webServerModule.isRestartRequested())
  {
    Log.println(F("\n\t--- Restarting System ---\n"));
    ESP.restart();
  }
#endif // MODULE_WEBSERVER_H

#if defined(MODULE_ANENOMETER_H)
  if (anenometerModule.isTriggered())
  {
    AnenometerMode();

    updatePanel();

    while (anenometerModule.isTriggered())
      ; // avoid multiple calls
  }
#endif // MODULE_ANENOMETER_H

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

    if (minutesDiff >= configModule.getUpdatePanelAdjustmentInterval())
    {
      Log.print(F("\n\t--- New Solar Panel Adjustment ---\n"));
      updatePanel();
    }
    else
    {
      Log.print(F("."));
    }
  }
}

// ----------------- Panel control functions ---------------------

void resetPanelPosition()
{
  Log.println(F("\n\t--- Resetting Solar Panel Position ---\n"));

  elevationController.moveToMaxElevation();
  if (azimuthController.moveFullLeft() < 0)
  {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment. "));
    errorMode();
  }

  Log.println(F("\n\t--- Solar Panel Position Reset ---\n"));
}

void updatePanel()
{
  Log.println(F("\n\t--- Updating Solar Panel Position ---\n"));
  lastPanelAdjustmentTime = rtcModule.getDateTimeUTC();
  DateTime lastPanelAdjustmentTimeLocal = rtcModule.getDateTimeLocal();

  currentTime.year = lastPanelAdjustmentTime.year();
  currentTime.month = lastPanelAdjustmentTime.month();
  currentTime.day = lastPanelAdjustmentTime.day();
  currentTime.hour = lastPanelAdjustmentTime.hour();
  currentTime.minute = lastPanelAdjustmentTime.minute();
  currentTime.second = lastPanelAdjustmentTime.second();

  SolTrack(currentTime, locationData, &solarPosition, configModule.getUseDegrees(), configModule.getUseNorthEqualsZero(), configModule.getComputeRefrEquatorial(), configModule.getComputeDistance());

  Log.println();

  rtcModule.printDateTime(lastPanelAdjustmentTime);

  Log.print(F("Corrected Azimuth: "));
  Log.print(solarPosition.azimuthRefract, 2);
  Log.print(F("°\tAltitude: "));
  Log.print(solarPosition.altitudeRefract, 2);
  Log.println(F("°"));

  float newelevation = elevationController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);
  float newazimuth = azimuthController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);

  if (newazimuth < 0)
  {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment. "));
    errorMode();
  }

#if defined(MODULE_MQTT_H)
  mqttModule.updateField(SOLAR_AZIMUTH, &solarPosition.azimuthRefract);
  mqttModule.updateField(SOLAR_ELEVATION, &solarPosition.altitudeRefract);
  mqttModule.updateField(PANEL_AZIMUTH, &newazimuth);
  mqttModule.updateField(PANEL_ELEVATION, &newelevation);
  mqttModule.updateField(LAST_PANEL_ADJUSTMENT_TIME, &lastPanelAdjustmentTimeLocal);
  DateTime nextPanelAdjustmentTimeLocal = lastPanelAdjustmentTimeLocal.unixtime() + configModule.getUpdatePanelAdjustmentInterval() * 60;
  mqttModule.updateField(NEXT_PANEL_ADJUSTMENT_TIME, &nextPanelAdjustmentTimeLocal);
#endif // MODULE_MQTT_H

  Log.println(F("\n\t--- Solar Panel Position Updated ---\n"));
}

// ----------------- Joystick control functions ---------------------

#if defined(JOYSTICK_CONTROLLER_H)
void initJoystick()
{
  joystickController.init();

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
  Log.println(F("\nEntering Joystick Mode"));

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

  Log.println(F("Exiting Joystick Mode"));
}
#endif // JOYSTICK_CONTROLLER_H
// ----------------- Anenometer control functions ---------------------

void AnenometerMode()
{
  Log.println(F("\nEntering Anenometer Mode"));

  elevationController.moveToMaxElevation();

  unsigned long countdownStart = millis();
  unsigned long countdownEnd = countdownStart + configModule.getAnenometerSafeDuration();

  while (millis() < countdownEnd)
  {
    if (anenometerModule.isTriggered())
    {
      Log.println(F("Anenometer triggered, resetting countdown"));

      countdownStart = millis();
      countdownEnd = countdownStart + configModule.getAnenometerSafeDuration();
    }
    delay(100);
  }

  Log.println(F("Exiting Anenometer Mode"));
}

// ----------------- ESP32 Error mode ---------------------

void errorMode()
{
  Log.println(F("\n\t--- Entering Error Mode ---\n"));
  elevationController.moveToMaxElevation();

#if defined(ESP32)
  Log.println(F("Entering Deep Sleep Mode"));
  esp_deep_sleep_start();
#endif // ESP32

  Log.println(F("\n\tEntering infinite loop"));
  while (true)
    delay(1000);
}