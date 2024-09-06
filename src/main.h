// SolarTracker.h

#ifndef SOLAR_TRACKER_H
#define SOLAR_TRACKER_H

#include <Arduino.h>
#include <RTClib.h>
#include <SPI.h>
#include "stdlib.h"
#include "SolTrack.h"
#include "ezButton.h"

#include "config.h"
#include "controllers/AzimuthController.h"
#include "controllers/ElevationController.h"
#include "controllers/JoystickController.h"

// Function Prototypes
void initRTC();
void resetPanelPosition();
void calibratePanel();
void updatePanel();
void initJoystick();
void JoystickMode();

#endif // SOLAR_TRACKER_H
