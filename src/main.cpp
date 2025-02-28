#include "main.h"

AzimuthController azimuthController;
ElevationController elevationController;
JoystickController joystickController;
AnenometerModule anenometerModule;
ConfigModule configModule;
MQTTModule mqttModule;
OTAModule otaModule;
RTCModule rtcModule;
WebServerModule webServerModule;
WifiModule wifiModule;
Logger Log;


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
    wifiModule.loop();
  
    mqttModule.loop();

    otaModule.loop();

    if (webServerModule.isRestartRequested()) {
      Log.println(F("\n\t--- Restarting System ---\n"));
      ESP.restart();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// -----------------------------------------------------------------------------
// Solar Panel Task (Core 1): Handles solar tracking, joystick, and anemometer modes
// -----------------------------------------------------------------------------
void solarTask(void* parameter) {
  Log.println("[SolarTask] Starting one-time calibration on Core 1...");

  calibratePanel();
  updatePanel();

  Log.println("[SolarTask] Calibration and initial update done. Entering main loop.");

  unsigned long lastPanelAdjustmentMillisLocal = millis();

  for (;;) {
    unsigned long currentMillis = millis();

    // Check Anenometer trigger
    if (anenometerModule.isTriggered()) {
      AnenometerMode();
      updatePanel();
      // Wait until the anemometer is no longer triggered
      while (anenometerModule.isTriggered()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }

    // Check for Joystick press
    if (joystickController.isPressed()) {
      JoystickMode();
      resetPanelPosition();
      updatePanel();
      // Wait until the joystick is released
      while (joystickController.isPressed()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }

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
  mqttModule.updateField(SOLAR_AZIMUTH, &solarPosition.azimuthRefract);
  mqttModule.updateField(SOLAR_ELEVATION, &solarPosition.altitudeRefract);
  mqttModule.updateField(PANEL_AZIMUTH, &newazimuth);
  mqttModule.updateField(PANEL_ELEVATION, &newelevation);
  mqttModule.updateField(LAST_PANEL_ADJUSTMENT_TIME, &lastPanelAdjustmentTimeLocal);
  DateTime nextPanelAdjustmentTimeLocal = lastPanelAdjustmentTimeLocal.unixtime() + configModule.getUpdatePanelAdjustmentInterval() * 60;
  mqttModule.updateField(NEXT_PANEL_ADJUSTMENT_TIME, &nextPanelAdjustmentTimeLocal);

  Log.println(F("\n\t--- Solar Panel Position Updated ---\n"));
}

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

  Log.println(F("Entering Deep Sleep Mode"));
  esp_deep_sleep_start();
}

// -----------------------------------------------------------------------------
// Setup and Loop
// -----------------------------------------------------------------------------
void setup() {
  delay(1000);
  configModule.begin();
  delay(1000);

  wifiModule.init();

  delay(1000);
  Log.begin(9600);
  configModule.printConfig();

  Log.println(F("\n\t--- System Initialization ---\n"));

  anenometerModule.init();
  rtcModule.init();
  mqttModule.init();
  otaModule.init();
  webServerModule.begin();

  initJoystick();

  locationData.latitude    = configModule.getSTLatitude();
  locationData.longitude   = configModule.getSTLongitude();
  locationData.pressure    = configModule.getSTPressure();
  locationData.temperature = configModule.getSTTemperature();

  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(solarTask, "SolarTask", 8192, NULL, 1, NULL, 1);

  Log.println(F("\n\t--- Setup() Done. Tasks Created. ---\n"));
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}