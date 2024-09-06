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
void resetPanelPosition();
void calibratePanel();
void updatePanel();
void JoystickMode();
void initializeJoystick();
void printDateTime(DateTime now);

#endif // SOLAR_TRACKER_H
