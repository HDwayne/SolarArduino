#include "ElevationController.h"

// Constructor
ElevationController::ElevationController(int motorEnPin, int motorPwmUpPin, int motorPwmDownPin, float maxAzimuth, float minAzimuth, float maxElevation, float minElevation, unsigned long timeThreshold, float actuatorSpeed, float actuatorLength)
    : motorPinEn(motorEnPin),
      motorPinPwmUp(motorPwmUpPin),
      motorPinPwmDown(motorPwmDownPin),
      motorController(motorEnPin, motorPwmUpPin, motorPwmDownPin),
      currentElevation(0),
      actuatorSpeed(actuatorSpeed),
      actuatorLength(actuatorLength),
      azimuthDegMax(maxAzimuth),
      azimuthDegMin(minAzimuth),
      elevationDegMax(maxElevation),
      elevationDegMin(minElevation),
      elevationTimeThreshold(timeThreshold)
{
}

// --------------------------------------------------

void ElevationController::calibrate()
{
  Serial.println(F("\n\t--- Starting Elevation Calibration Procedure ---\n"));

  actuatorFullTravelTime = (actuatorLength / actuatorSpeed) * 1000.0;
  Serial.print(F("\tThe actuator full travel time is: "));
  Serial.print(actuatorFullTravelTime);
  Serial.println(F(" ms"));

  degreesPerMs = (elevationDegMax - elevationDegMin) / actuatorFullTravelTime;
  Serial.print(F("\tDegrees per millisecond: "));
  Serial.print(degreesPerMs, 6);
  Serial.println(F("°/ms\n"));

  moveToMaxElevation();

  Serial.println(F("\n\t--- Elevation Calibration Procedure Completed ---\n"));
}

void ElevationController::moveToAngle(float targetAzimuth, float targetElevation)
{
  Serial.println(F("\n\t--- Adjusting Elevation ---\n"));

  if (targetElevation > elevationDegMax || targetElevation < elevationDegMin)
  {
    Serial.println(F("[INFO] Solar elevation is out of tracking range. Cannot adjust elevation."));

    if (targetAzimuth > azimuthDegMax || targetAzimuth < azimuthDegMin)
    {
      Serial.println(F("[INFO] Solar azimuth is also out of tracking range."));
      if (currentElevation != elevationDegMax)
        moveToMaxElevation();
    }
    else
    {
      Serial.println(F("[INFO] Adjusting azimuth to the nearest tracking position."));
      if (currentElevation != elevationDegMin)
        moveToMinElevation();
    }

    return;
  }

  float azimuthDifference = fabs(targetElevation - currentElevation);
  float timeToMove = azimuthDifference / degreesPerMs;

  if (timeToMove < elevationTimeThreshold)
  {
    Serial.println(F("[INFO] Time to move is less than the minimal required time. Cannot adjust elevation."));
    return;
  }

  Serial.print(F("[INFO] Elevation difference: "));
  Serial.print(azimuthDifference, 2);
  Serial.print(F("°. Time to move: "));
  Serial.print(timeToMove);
  Serial.println(F(" ms"));

  motorController.Enable();

  if (targetElevation > currentElevation)
  {
    Serial.println(F("[ADJUST] Moving actuator up."));
    currentElevation += azimuthDifference;
    startActuatorUp();
    delay(timeToMove);
  }
  else
  {
    Serial.println(F("[ADJUST] Moving actuator down."));
    currentElevation -= azimuthDifference;
    startActuatorDown();
    delay(timeToMove);
  }

  stopActuator();
  motorController.Disable();

  Serial.print(F("[ADJUST] Elevation adjusted. Current elevation: "));
  Serial.println(currentElevation, 2);

  Serial.println(F("\n\t--- End of Elevation Adjustment ---\n"));
}

void ElevationController::moveToMaxElevation()
{
  Serial.println(F("-> Moving to max elevation position"));
  motorController.Enable();

  startActuatorUp();
  delay(actuatorFullTravelTime);
  stopActuator();

  motorController.Disable();

  currentElevation = elevationDegMax;
  Serial.println(F("-> Max elevation position reached"));
}

void ElevationController::moveToMinElevation()
{
  Serial.println(F("-> Moving to min elevation position"));
  motorController.Enable();

  startActuatorDown();
  delay(actuatorFullTravelTime);
  stopActuator();

  motorController.Disable();

  currentElevation = elevationDegMin;
  Serial.println(F("-> Min elevation position reached"));
}

// --------------------------------------------------

void ElevationController::startActuatorUp()
{
  Serial.println(F("[ACTUATOR] Starting actuator. Direction: up."));
  motorController.TurnLeft(ELEVATION_MOTOR_PWM_SPEED);
}

void ElevationController::startActuatorDown()
{
  Serial.println(F("[ACTUATOR] Starting actuator. Direction: down."));
  motorController.TurnRight(ELEVATION_MOTOR_PWM_SPEED);
}

void ElevationController::stopActuator()
{
  Serial.println(F("[ACTUATOR] Stopping actuator."));
  motorController.Stop();
}
