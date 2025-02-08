#include "main.h"

#if defined(ESP32)
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
#endif

// -----------------------------------------------------------------------------
// Global Structures and Variables
// -----------------------------------------------------------------------------
struct STTime currentTime;       // Date and time structure
struct STLocation locationData;  // Geographic location structure
struct STPosition solarPosition; // Solar position structure

DateTime lastPanelAdjustmentTime; 
unsigned long lastPanelAdjustmentMillis = 0;
const unsigned long panelAdjustmentIntervalMillis = 60000; // Check every minute

// -----------------------------------------------------------------------------
// Communication Task (Core 0): Handles WiFi, MQTT, OTA, WebServer, etc.
// -----------------------------------------------------------------------------
void wifiTask(void* parameter) {
  for (;;) {
    #if defined(MODULE_MQTT_H)
      mqttModule.loop();
    #endif // MODULE_MQTT_H

    #if defined(MODULE_OTA_H)
      otaModule.loop();
    #endif // MODULE_OTA_H

    #if defined(MODULE_WEBSERVER_H)
      if (webServerModule.isRestartRequested()) {
        Log.println(F("\n\t--- Restarting System ---\n"));
        ESP.restart();
      }
    #endif // MODULE_WEBSERVER_H

    // Yield to other tasks
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// -----------------------------------------------------------------------------
// Solar Panel Task (Core 1): Handles solar tracking, joystick, and anemometer modes
// -----------------------------------------------------------------------------
void solarTask(void* parameter) {
  unsigned long lastPanelAdjustmentMillisLocal = millis();

  for (;;) {
    unsigned long currentMillis = millis();

    // Check Anenometer trigger
    #if defined(MODULE_ANENOMETER_H)
    if (anenometerModule.isTriggered()) {
      AnenometerMode();
      updatePanel();
      // Wait until the anemometer is no longer triggered
      while (anenometerModule.isTriggered()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }
    #endif

    // Check for Joystick press
    #if defined(JOYSTICK_CONTROLLER_H)
    if (joystickController.isPressed()) {
      JoystickMode();
      resetPanelPosition();
      updatePanel();
      // Wait until the joystick is released
      while (joystickController.isPressed()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }
    #endif

    // Periodic panel adjustment
    if (currentMillis - lastPanelAdjustmentMillisLocal >= panelAdjustmentIntervalMillis) {
      lastPanelAdjustmentMillisLocal = currentMillis;
      
      DateTime now = rtcModule.getDateTimeUTC();
      uint32_t secc = now.unixtime() - lastPanelAdjustmentTime.unixtime();
      uint16_t minutesDiff = secc / 60;

      if (minutesDiff >= configModule.getUpdatePanelAdjustmentInterval()) {
        Log.println(F("\n\t--- New Solar Panel Adjustment ---\n"));
        updatePanel();
      } else {
        Log.print(F("."));
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// -----------------------------------------------------------------------------
// System Functions
// -----------------------------------------------------------------------------

// Initializes the RTC and adjusts it if power was lost.
void initRTC() {
  rtcModule.init();
  if (rtcModule.lostPower()) {
    #if defined(ESP32)
      rtcModule.adjustFromNTP();
    #else
      Log.println(F("[ERROR] RTC module lost power and no NTP server available. Please set the date and time manually."));
      errorMode();
    #endif
  }
}

// Calibrates the solar panel by initializing both controllers.
void calibratePanel() {
  elevationController.init();
  if (azimuthController.init() < 0) {
    Log.println(F("[ERROR] An error occurred during azimuth calibration."));
    errorMode();
  }
}

// Resets the solar panel to a default (or safe) position.
void resetPanelPosition() {
  Log.println(F("\n\t--- Resetting Solar Panel Position ---\n"));

  elevationController.moveToMaxElevation();
  if (azimuthController.moveFullLeft() < 0) {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment."));
    errorMode();
  }

  Log.println(F("\n\t--- Solar Panel Position Reset ---\n"));
}

// Updates the solar panel's position based on the current solar calculations.
void updatePanel() {
  Log.println(F("\n\t--- Updating Solar Panel Position ---\n"));
  lastPanelAdjustmentTime = rtcModule.getDateTimeUTC();
  DateTime lastPanelAdjustmentTimeLocal = rtcModule.getDateTimeLocal();

  // Update the current time structure for solar calculations
  currentTime.year   = lastPanelAdjustmentTime.year();
  currentTime.month  = lastPanelAdjustmentTime.month();
  currentTime.day    = lastPanelAdjustmentTime.day();
  currentTime.hour   = lastPanelAdjustmentTime.hour();
  currentTime.minute = lastPanelAdjustmentTime.minute();
  currentTime.second = lastPanelAdjustmentTime.second();

  // Calculate the solar position
  SolTrack(currentTime, locationData, &solarPosition,
           configModule.getUseDegrees(),
           configModule.getUseNorthEqualsZero(),
           configModule.getComputeRefrEquatorial(),
           configModule.getComputeDistance());

  Log.println();
  rtcModule.printDateTime(lastPanelAdjustmentTime);
  Log.print(F("Corrected Azimuth: "));
  Log.print(solarPosition.azimuthRefract, 2);
  Log.print(F("°\tAltitude: "));
  Log.print(solarPosition.altitudeRefract, 2);
  Log.println(F("°"));

  // Move the panel to the calculated solar position
  float newelevation = elevationController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);
  float newazimuth   = azimuthController.moveToAngle(solarPosition.azimuthRefract, solarPosition.altitudeRefract);

  if (newazimuth < 0) {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment."));
    errorMode();
  }

  // Optionally update MQTT fields with new values
  #if defined(MODULE_MQTT_H)
    mqttModule.updateField(SOLAR_AZIMUTH, &solarPosition.azimuthRefract);
    mqttModule.updateField(SOLAR_ELEVATION, &solarPosition.altitudeRefract);
    mqttModule.updateField(PANEL_AZIMUTH, &newazimuth);
    mqttModule.updateField(PANEL_ELEVATION, &newelevation);
    mqttModule.updateField(LAST_PANEL_ADJUSTMENT_TIME, &lastPanelAdjustmentTimeLocal);
    DateTime nextPanelAdjustmentTimeLocal = lastPanelAdjustmentTimeLocal.unixtime() + configModule.getUpdatePanelAdjustmentInterval() * 60;
    mqttModule.updateField(NEXT_PANEL_ADJUSTMENT_TIME, &nextPanelAdjustmentTimeLocal);
  #endif

  Log.println(F("\n\t--- Solar Panel Position Updated ---\n"));
}

#if defined(JOYSTICK_CONTROLLER_H)
// Initializes the joystick and sets up callbacks for directional commands.
void initJoystick() {
  joystickController.init();

  joystickController.setOnDown([]() {
    azimuthController.startMotorRight();
  });

  joystickController.setOnUp([]() {
    azimuthController.startMotorLeft();
  });

  joystickController.setOnLeaveLR([]() {
    elevationController.stopActuator();
  });

  joystickController.setOnLeaveUD([]() {
    azimuthController.stopMotor();
  });

  joystickController.setOnLeft([]() {
    elevationController.startActuatorDown();
  });

  joystickController.setOnRight([]() {
    elevationController.startActuatorUp();
  });
}

// Allows manual control of the panel via the joystick.
void JoystickMode() {
  Log.println(F("\nEntering Joystick Mode"));

  azimuthController.enableMotor();
  elevationController.enableMotor();

  joystickController.clearCommand();

  while (!joystickController.isPressed()) {
    joystickController.executeCommand();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield for responsiveness
  }

  azimuthController.stopMotor();
  azimuthController.disableMotor();

  elevationController.stopActuator();
  elevationController.disableMotor();

  Log.println(F("Exiting Joystick Mode"));
}
#endif // JOYSTICK_CONTROLLER_H

// Puts the panel in a safe mode in response to an anemometer trigger.
void AnenometerMode() {
  Log.println(F("\nEntering Anenometer Mode"));

  elevationController.moveToMaxElevation();

  unsigned long countdownStart = millis();
  unsigned long countdownEnd = countdownStart + configModule.getAnenometerSafeDuration();

  while (millis() < countdownEnd) {
    if (anenometerModule.isTriggered()) {
      Log.println(F("Anenometer triggered, resetting countdown"));
      countdownStart = millis();
      countdownEnd = countdownStart + configModule.getAnenometerSafeDuration();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  Log.println(F("Exiting Anenometer Mode"));
}

// Handles error conditions. For ESP32, it enters deep sleep.
void errorMode() {
  Log.println(F("\n\t--- Entering Error Mode ---\n"));
  elevationController.moveToMaxElevation();

  #if defined(ESP32)
    Log.println(F("Entering Deep Sleep Mode"));
    esp_deep_sleep_start();
  #endif

  Log.println(F("\n\tEntering infinite loop"));
  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// -----------------------------------------------------------------------------
// Setup and Loop
// -----------------------------------------------------------------------------
void setup() {
  delay(1000);
  configModule.begin();
  delay(1000);

  #if defined(MODULE_WIFI_H)
    wifiModule.init();
  #endif

  delay(1000);
  Log.begin(9600);
  configModule.printConfig();

  Log.println(F("\n\t--- System Initialization ---\n"));

  #if defined(MODULE_ANENOMETER_H)
    anenometerModule.init();
  #endif

  initRTC();

  #if defined(MODULE_MQTT_H)
    mqttModule.init();
  #endif

  #if defined(MODULE_OTA_H)
    otaModule.init();
  #endif

  #if defined(MODULE_WEBSERVER_H)
    webServerModule.begin();
  #endif

  #if defined(JOYSTICK_CONTROLLER_H)
    initJoystick();
  #endif

  // Set location data for solar calculations
  locationData.latitude    = configModule.getSTLatitude();
  locationData.longitude   = configModule.getSTLongitude();
  locationData.pressure    = configModule.getSTPressure();
  locationData.temperature = configModule.getSTTemperature();

  // Calibrate the solar panel (initialize both elevation and azimuth controllers)
  calibratePanel();

  // Update the solar panel to the correct position initially
  updatePanel();

  Log.println(F("\n\t--- System Initialization Completed ---\n"));

  #if defined(ESP32)
    // Create FreeRTOS tasks on separate cores
    // Communications on core 0 (typically used by WiFi)
    xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4096, NULL, 1, NULL, 0);
    // Solar panel operations on core 1
    xTaskCreatePinnedToCore(solarTask, "SolarTask", 8192, NULL, 1, NULL, 1);
  #endif
}

void loop() {
  // The main loop is empty because all operations are handled in the FreeRTOS tasks.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
