/*
  SolTrack: a simple, free, fast and accurate C routine to compute the position of the Sun

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

#ifndef __SOLTRACK_H
#define __SOLTRACK_H

#include "stdio.h"
#include "math.h"

// Constants:

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530717958647693
#endif

#define MPI 3.14159265358979323846e6 // One Megapi...
#define R2D 57.2957795130823208768   // Radians to degrees conversion factor
#define R2H 3.81971863420548805845   // Radians to hours conversion factor

// Structs:

/// @brief Date and time to compute the Sun's position for, in UT
struct STTime
{
  int year, month, day, hour, minute;
  double second;
};

/// @brief Location to compute the Sun's position for
struct STLocation
{
  double longitude, latitude;
  double sinLat, cosLat;
  double pressure, temperature;
};

/// @brief Position of the Sun
struct STPosition
{
  double julianDay, tJD, tJC, tJC2, UT;
  double longitude, distance;
  double obliquity, cosObliquity, nutationLon;
  double rightAscension, declination, hourAngle, agst;
  double altitude, altitudeRefract, azimuthRefract;
  double hourAngleRefract, declinationRefract;
};

/// @brief Rise and set data for the Sun
struct STRiseSet
{
  double riseTime, transitTime, setTime;
  double riseAzimuth, transitAltitude, setAzimuth;
};

// Function prototypes:
void SolTrack(struct STTime time, struct STLocation location, struct STPosition *position, int useDegrees, int useNorthEqualsZero, int computeRefrEquatorial, int computeDistance);
double computeJulianDay(int year, int month, int day, int hour, int minute, double second);
void computeLongitude(int computeDistance, struct STPosition *position);
void convertEclipticToEquatorial(double longitude, double cosObliquity, double *rightAscension, double *declination);
void convertEquatorialToHorizontal(struct STLocation location, struct STPosition *position);
void eq2horiz(double sinLat, double cosLat, double longitude, double rightAscension, double declination, double agst, double *azimuth, double *sinAlt);
void convertHorizontalToEquatorial(double sinLat, double cosLat, double azimuth, double altitude, double *hourAngle, double *declination);
void setNorthToZero(double *azimuth, double *hourAngle, int computeRefrEquatorial);
void convertRadiansToDegrees(double *longitude, double *rightAscension, double *declination,
                             double *altitude, double *azimuthRefract, double *altitudeRefract,
                             double *hourAngle, double *declinationRefract, int computeRefrEquatorial);
double STatan2(double y, double x);
#endif
