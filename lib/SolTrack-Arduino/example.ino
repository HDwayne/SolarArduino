/*
  SolTrack: a simple, free, fast and accurate Arduino routine to compute the position of the Sun
  
  Copyright (c) 2014-2021  Marc van der Sluys, Paul van Kan and Jurgen Reintjes,
  Sustainable Energy research group, HAN University of applied sciences, Arnhem, The Netherlands
   
  This file is part of the SolTrack package, see: http://soltrack.sourceforge.net
  SolTrack is derived from libTheSky (http://libthesky.sourceforge.net) under the terms of the GPL v.3
  
  This is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General
  Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.
  
  This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.
  
  You should have received a copy of the GNU Lesser General Public License along with this code.  If not, see 
  <http://www.gnu.org/licenses/>.
*/


#include <example.h>
#include <SolTrack.h>
#include "stdlib.h"

// Global variables and structs:
int useDegrees = 1;             // Input (geographic position) and output are in degrees
int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
int computeRefrEquatorial = 0;  // Compure refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
int computeDistance = 0;        // Compute the distance to the Sun in AU: 0-no, 1-yes

struct STTime time;               // Struct for date and time variables
struct STLocation loc;            // Struct for geographic location variables



void setup() {
  Serial.begin(9600);
	
	// Set location for HAN University of applied sciences, Arnhem, The Netherlands:
	loc.longitude   =  5.950270;  // HAN University of applied sciences, Arnhem, The Netherlands
	loc.latitude    = 51.987380;
	loc.pressure    = 101.0;      // Atmospheric pressure in kPa
	loc.temperature = 283.0;      // Atmospheric temperature in K
}

void loop() {
	solTrack_computeAndPrintSunPos(2045, 7, 16, 6, 2, 49.217348);  // Time in UT!
	
	while(1){}  // Stop (actually, go into an infinite loop)
}


/**
 * @brief   Example function to compute the position of the Sun and print it to Serial.
 *
 * @param [in]    year    Year to compute the Sun position for (UT).
 * @param [in]    month   Month to compute the Sun position for (UT).
 * @param [in]    day     Day to compute the Sun position for (UT).
 * @param [in]    hour    Hour to compute the Sun position for (UT).
 * @param [in]    minute  Minute to compute the Sun position for (UT).
 * @param [in]    second  Second to compute the Sun position for (UT).
 *
 * Example Usage:
 * @code solTrack_computeAndPrintSunPos(2045, 7, 16, 6, 2, 49.217348);  // Time in UT!@endcode
 */

void solTrack_computeAndPrintSunPos(int year, int month, int day, int hour, int minute, double second) {
	// Set (UT!) date and time:
  time.year   = year;
  time.month  = month;
  time.day    = day;
  time.hour   = hour;
  time.minute = minute;
  time.second = second;
	
  // Compute Sun position:
  struct STPosition pos;
  SolTrack(time, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);
	
  // Write formatted output to serial connection:
	char outputLine[256], secStr[7], JDstr[13], agstStr[11], azStr[9],altStr[8];
	dtostrf(time.second,         6, 3, secStr);   //  6 digits, 3 decimals
	dtostrf(pos.julianDay,      12, 6, JDstr);    // 14 digits, 6 decimals
	dtostrf(pos.agst*R2H,       10, 7, agstStr);  // 10 digits, 7 decimals
	dtostrf(pos.azimuthRefract,  8, 3, azStr);    //  8 digits, 3 decimals
	dtostrf(pos.altitudeRefract, 7, 3, altStr);   //  8 digits, 3 decimals
	
	sprintf(outputLine, "%d-%2.2d-%2.2d %2.2d:%2.2d:%s %s %s %s %s %s",
					time.year, time.month, time.day, time.hour,time.minute,secStr,
					JDstr, agstStr, azStr,altStr, "     ");
	
	Serial.println(outputLine);
}


