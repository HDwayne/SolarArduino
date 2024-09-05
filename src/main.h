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

// Function Prototypes
void resetPanelPosition();
void calibratePanel();
void updatePanel();
void joyStick();
void updateSunPos();
void printSunPos();
void printDateTime(DateTime now);

#endif // SOLAR_TRACKER_H
