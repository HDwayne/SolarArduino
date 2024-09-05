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

// Structs for SolTrack
extern struct STTime time;              // Struct for date and time variables
extern struct STLocation locationData;  // Struct for geographic locationDataation variables
extern struct STPosition solarPosition; // Struct for solar position variables

// Time variables
extern DateTime lastPanelAdjustmentTime; // Last time the solar panel was adjusted

// RTC Module
extern RTC_DS1307 rtc;

// Function Prototypes
void resetPanelPosition();
void calibratePanel();
void updatePanel();
void joyStick();
void updateSunPos();
void printSunPos();
void printDateTime(DateTime now);

#endif // SOLAR_TRACKER_H
