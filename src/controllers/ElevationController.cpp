#include "ElevationController.h"

#include "utils/Logger.h"
#include "modules/ConfigModule.h"
#include "modules/MQTTModule.h"

extern ConfigModule configModule;
extern Logger Log;
extern MQTTModule mqttModule;

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

  Log.println(F("\n\t--- Starting Elevation Initialization Procedure ---\n"));

  if (configModule.getForceTimeFullTravel() > 0)
  {
    actuatorFullTravelTime = configModule.getForceTimeFullTravel() * 1000.0;
  }
  else
  {
    actuatorFullTravelTime = (actuatorLength / actuatorSpeed) * 1000.0;
  }

  Log.print(F("\tThe actuator full travel time is: "));
  Log.print(actuatorFullTravelTime);
  Log.println(F(" ms"));

  degreesPerMs = (elevationDegMax - elevationDegMin) / actuatorFullTravelTime;
  Log.print(F("\tDegrees per millisecond: "));
  Log.print(degreesPerMs, 6);
  Log.println(F("°/ms\n"));

  moveToMaxElevation();

  Log.println(F("\n\t--- Elevation Initialization Procedure Completed ---\n"));
}

float ElevationController::moveToAngle(float targetAzimuth, float targetElevation)
{
  Log.println(F("\n\t--- Adjusting Elevation ---\n"));

  if (targetElevation > elevationDegMax || targetElevation < elevationDegMin)
  {
    Log.println(F("[INFO] Solar elevation is out of tracking range. Cannot adjust elevation."));

    if (targetAzimuth > azimuthDegMax || targetAzimuth < azimuthDegMin || targetElevation <= 0.0)
    {
      Log.println(F("[INFO] Solar azimuth is also out of tracking range or sun below the horizon. Stopping elevation adjustment."));
      if (currentElevation != elevationDegMax)
        moveToMaxElevation();
    }
    else
    {
      Log.println(F("[INFO] Adjusting azimuth to the nearest tracking position."));
      if (currentElevation != elevationDegMin)
        moveToMinElevation();
    }

    mqttModule.publishElevationState();
    return currentElevation;
  }

  float azimuthDifference = fabs(targetElevation - currentElevation);
  float timeToMove = azimuthDifference / degreesPerMs;

  if (timeToMove < elevationTimeThreshold)
  {
    Log.println(F("[INFO] Time to move is less than the minimal required time. Cannot adjust elevation."));
    mqttModule.publishElevationState();
    return currentElevation;
  }

  Log.print(F("[INFO] Elevation difference: "));
  Log.print(azimuthDifference, 2);
  Log.print(F("°. Time to move: "));
  Log.print(timeToMove);
  Log.println(F(" ms"));

  motorController->Enable();

  if (targetElevation > currentElevation)
  {
    Log.println(F("[ADJUST] Moving actuator up."));
    currentElevation += azimuthDifference;
    startActuatorUp();
    delay(timeToMove);
  }
  else
  {
    Log.println(F("[ADJUST] Moving actuator down."));
    currentElevation -= azimuthDifference;
    startActuatorDown();
    delay(timeToMove);
  }

  stopActuator();
  motorController->Disable();

  Log.print(F("[ADJUST] Elevation adjusted. Current elevation: "));
  Log.println(currentElevation, 2);
  mqttModule.publishElevationState();
  Log.println(F("\n\t--- End of Elevation Adjustment ---\n"));

  return currentElevation;
}

void ElevationController::moveToMaxElevation()
{
  Log.println(F("-> Moving to max elevation position"));
  motorController->Enable();

  startActuatorUp();
  delay(actuatorFullTravelTime);
  stopActuator();

  motorController->Disable();

  currentElevation = elevationDegMax;
  mqttModule.publishElevationState();
  Log.println(F("-> Max elevation position reached"));
}

void ElevationController::moveToMinElevation()
{
  Log.println(F("-> Moving to min elevation position"));
  motorController->Enable();

  startActuatorDown();
  delay(actuatorFullTravelTime);
  stopActuator();

  motorController->Disable();

  currentElevation = elevationDegMin;
  mqttModule.publishElevationState();
  Log.println(F("-> Min elevation position reached"));
}

// --------------------------------------------------

void ElevationController::startActuatorUp()
{
  Log.println(F("[ACTUATOR] Starting actuator. Direction: up."));
  motorController->TurnLeft(motorPwmSpeed);
  mqttModule.publishElevationController("Moving up");
}

void ElevationController::startActuatorDown()
{
  Log.println(F("[ACTUATOR] Starting actuator. Direction: down."));
  motorController->TurnRight(motorPwmSpeed);
  mqttModule.publishElevationController("Moving down");
}

void ElevationController::stopActuator()
{
  Log.println(F("[ACTUATOR] Stopping actuator."));
  motorController->Stop();
  mqttModule.publishElevationController("Stopped");
}
