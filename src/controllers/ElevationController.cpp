#include "ElevationController.h"
#include "modules/ConfigModule.h"

ElevationController elevationController;

// ----------------- Elevation Controller Constructor -----------------

ElevationController::ElevationController() {}

// --------------------------------------------------

void ElevationController::init()
{
  motorPinEn = configModule.getElevationMotorPinEn();
  motorPinPwmUp = configModule.getElevationMotorPwmPinU();
  motorPinPwmDown = configModule.getElevationMotorPwmPinD();
  motorPwmSpeed = configModule.getElevationMotorPWMSpeed();
  motorController = new BTS7960(motorPinEn, motorPinPwmUp, motorPinPwmDown);
  currentElevation = 0.0;
  actuatorSpeed = configModule.getElevationActuatorSpeed();
  actuatorLength = configModule.getElevationActuatorLength();
  actuatorFullTravelTime = 0;
  degreesPerMs = 0.0;
  azimuthDegMax = configModule.getAzimuthDegMax();
  azimuthDegMin = configModule.getAzimuthDegMin();
  elevationDegMax = configModule.getElevationDegMax();
  elevationDegMin = configModule.getElevationDegMin();
  elevationTimeThreshold = configModule.getElevationTimeThreshold();

  Serial.println(F("\n\t--- Starting Elevation Initialization Procedure ---\n"));

  if (configModule.getForceTimeFullTravel() > 0)
  {
    actuatorFullTravelTime = configModule.getForceTimeFullTravel() * 1000.0;
  }
  else
  {
    actuatorFullTravelTime = (actuatorLength / actuatorSpeed) * 1000.0;
  }

  Serial.print(F("\tThe actuator full travel time is: "));
  Serial.print(actuatorFullTravelTime);
  Serial.println(F(" ms"));

  degreesPerMs = (elevationDegMax - elevationDegMin) / actuatorFullTravelTime;
  Serial.print(F("\tDegrees per millisecond: "));
  Serial.print(degreesPerMs, 6);
  Serial.println(F("°/ms\n"));

  moveToMaxElevation();

  Serial.println(F("\n\t--- Elevation Initialization Procedure Completed ---\n"));
}

float ElevationController::moveToAngle(float targetAzimuth, float targetElevation)
{
  Serial.println(F("\n\t--- Adjusting Elevation ---\n"));

  if (targetElevation > elevationDegMax || targetElevation < elevationDegMin)
  {
    Serial.println(F("[INFO] Solar elevation is out of tracking range. Cannot adjust elevation."));

    if (targetAzimuth > azimuthDegMax || targetAzimuth < azimuthDegMin || targetElevation <= 0.0)
    {
      Serial.println(F("[INFO] Solar azimuth is also out of tracking range or sun below the horizon. Stopping elevation adjustment."));
      if (currentElevation != elevationDegMax)
        moveToMaxElevation();
    }
    else
    {
      Serial.println(F("[INFO] Adjusting azimuth to the nearest tracking position."));
      if (currentElevation != elevationDegMin)
        moveToMinElevation();
    }

    return currentElevation;
  }

  float azimuthDifference = fabs(targetElevation - currentElevation);
  float timeToMove = azimuthDifference / degreesPerMs;

  if (timeToMove < elevationTimeThreshold)
  {
    Serial.println(F("[INFO] Time to move is less than the minimal required time. Cannot adjust elevation."));
    return currentElevation;
  }

  Serial.print(F("[INFO] Elevation difference: "));
  Serial.print(azimuthDifference, 2);
  Serial.print(F("°. Time to move: "));
  Serial.print(timeToMove);
  Serial.println(F(" ms"));

  motorController->Enable();

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
  motorController->Disable();

  Serial.print(F("[ADJUST] Elevation adjusted. Current elevation: "));
  Serial.println(currentElevation, 2);

  Serial.println(F("\n\t--- End of Elevation Adjustment ---\n"));

  return currentElevation;
}

void ElevationController::moveToMaxElevation()
{
  Serial.println(F("-> Moving to max elevation position"));
  motorController->Enable();

  startActuatorUp();
  delay(actuatorFullTravelTime);
  stopActuator();

  motorController->Disable();

  currentElevation = elevationDegMax;
  Serial.println(F("-> Max elevation position reached"));
}

void ElevationController::moveToMinElevation()
{
  Serial.println(F("-> Moving to min elevation position"));
  motorController->Enable();

  startActuatorDown();
  delay(actuatorFullTravelTime);
  stopActuator();

  motorController->Disable();

  currentElevation = elevationDegMin;
  Serial.println(F("-> Min elevation position reached"));
}

// --------------------------------------------------

void ElevationController::startActuatorUp()
{
  Serial.println(F("[ACTUATOR] Starting actuator. Direction: up."));
  motorController->TurnLeft(motorPwmSpeed);
}

void ElevationController::startActuatorDown()
{
  Serial.println(F("[ACTUATOR] Starting actuator. Direction: down."));
  motorController->TurnRight(motorPwmSpeed);
}

void ElevationController::stopActuator()
{
  Serial.println(F("[ACTUATOR] Stopping actuator."));
  motorController->Stop();
}
