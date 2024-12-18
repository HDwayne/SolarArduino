// SolarTracker.h

#ifndef SOLAR_TRACKER_H
#define SOLAR_TRACKER_H

#include <Arduino.h>
#include <RTClib.h>
#include <SPI.h>
#include "stdlib.h"
#include "SolTrack.h"
#include "ezButton.h"

#include "controllers/AzimuthController.h"
#include "controllers/ElevationController.h"
#include "controllers/JoystickController.h"

#include "modules/ConfigModule.h"
#include "modules/RTCModule.h"
#include "modules/AnemometerModule.h"

#if defined(ESP32)
#include "esp_sleep.h"
#include "modules/WifiModule.h"
#include "modules/MQTTModule.h"
#include "modules/OTAModule.h"
#include "modules/WebServerModule.h"
#endif

#include "utils/Logger.h"

// Function Prototypes
void initRTC();
void resetPanelPosition();
void calibratePanel();
void updatePanel();
void initJoystick();
void JoystickMode();
void AnenometerMode();
void errorMode();

#endif // SOLAR_TRACKER_H
