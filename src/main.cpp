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

  // Initialize Modules
  configModule.begin();

#if defined(MODULE_ANENOMETER_H)
  anenometerModule.init();
#endif // MODULE_ANENOMETER_H

#if defined(MODULE_WIFI_H)
  wifiModule.init();
#endif // MODULE_WIFI_H

  rtcModule.init();
  if (rtcModule.lostPower())
  {
#if defined(ESP32)
    rtcModule.adjustFromNTP();
#else
    Serial.println(F("[ERROR] RTC module lost power and no NTP server available. Please set the date and time manually."));
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

  // initialize joystick
  initJoystick();

  // Set initial location data for solar calculations
  locationData.latitude = configModule.getSTLatitude();
  locationData.longitude = configModule.getSTLongitude();
  locationData.pressure = configModule.getSTPressure();
  locationData.temperature = configModule.getSTTemperature();

  // Calibrate the solar panel
  elevationController.init();
  if (azimuthController.init() < 0)
  {
    Serial.println(F("[ERROR] An error occurred during azimuth calibration. "));
    errorMode();
  }

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

#if defined(MODULE_OTA_H)
  otaModule.loop();
#endif // MODULE_OTA_H

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

  SolTrack(currentTime, locationData, &solarPosition, configModule.getUseDegrees(), configModule.getUseNorthEqualsZero(), configModule.getComputeRefrEquatorial(), configModule.getComputeDistance());

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
  DateTime nextPanelAdjustmentTimeLocal = lastPanelAdjustmentTimeLocal.unixtime() + configModule.getUpdatePanelAdjustmentInterval() * 60;
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
  unsigned long countdownEnd = countdownStart + configModule.getAnenometerSafeDuration();

  while (millis() < countdownEnd)
  {
    if (anenometerModule.isTriggered())
    {
      Serial.println(F("Anenometer triggered, resetting countdown"));

      countdownStart = millis();
      countdownEnd = countdownStart + configModule.getAnenometerSafeDuration();
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