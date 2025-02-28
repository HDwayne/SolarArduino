#include "AzimuthController.h"

#include "utils/Logger.h"
#include "modules/ConfigModule.h"

extern ConfigModule configModule;
extern Logger Log;

// ----------------- Azimuth Controller Constructor -----------------

AzimuthController::AzimuthController() {}

// ----------------- Azimuth control functions -----------------

int8_t AzimuthController::init()
{
  motorPinEn = configModule.getAzimuthMotorPinEn();
  motorPinPwmL = configModule.getAzimuthMotorPwmPinL();
  motorPinPwmR = configModule.getAzimuthMotorPwmPinR();
  motorPwmSpeed = configModule.getAzimuthMotorPWMSpeed();
  limitSwitchPin = configModule.getAzimuthLimitSwitchPin();
  motorController = new BTS7960(motorPinEn, motorPinPwmL, motorPinPwmR);
  limitSwitch = new ezButton(limitSwitchPin);
  limitSwitch->setDebounceTime(100);
  fullRotationDuration = configModule.getAzimuthTimeMaxBeforeCalibration();
  currentAzimuth = 0.0;
  degreesPerMs = 0.0;
  azimuthDegMax = configModule.getAzimuthDegMax();
  azimuthDegMin = configModule.getAzimuthDegMin();
  azimuthTimeThreshold = configModule.getAzimuthTimeThreshold();

  Log.println(F("\n\t--- Starting Initialization of Azimuth Controller ---\n"));

  moveFullRight();

  if (isError)
  {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment. Calibration failed."));
    return -1;
  }

  delay(3000);

  unsigned long pressStartTime = millis();
  moveFullLeft();

  if (isError)
  {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment. Calibration failed."));
    return -1;
  }

  fullRotationDuration = millis() - pressStartTime;
  Log.print(F("\n\tCalibration Duration: "));
  Log.print(fullRotationDuration);
  Log.println(F(" ms"));

  // Calculate degrees per millisecond
  degreesPerMs = 360.0 / fullRotationDuration;
  Log.print(F("\tDegrees per millisecond: "));
  Log.print(degreesPerMs, 6);
  Log.println(F("°/ms\n"));

  Log.println(F("\n\t--- Calibration procedure completed. ---\n"));

  return 0;
}

int8_t AzimuthController::moveFullLeft()
{
  Log.println(F("-> Moving to full left position"));
  motorController->Enable();

  startMotorLeft();
  waitForLimitSwitch();
  stopMotor();

  motorController->Disable();

  if (isError)
  {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment. "));
    return -1;
  }

  currentAzimuth = 0.0;
  Log.println(F("-> Full left position reached"));
  return 0;
}

int8_t AzimuthController::moveFullRight()
{
  Log.println(F("-> Moving to full right position"));
  motorController->Enable();

  startMotorRight();
  waitForLimitSwitch();
  stopMotor();

  motorController->Disable();

  if (isError)
  {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment. "));
    return -1;
  }

  currentAzimuth = 360.0;
  Log.println(F("-> Full right position reached"));
  return 0;
}

/*
Function to move the solar panel to a specific azimuth angle.
Returns the current azimuth angle after adjustment.
Returns -1.0 if an error occurred.
*/
float AzimuthController::moveToAngle(float targetAzimuth, float targetElevation)
{
  Log.println(F("\n\t--- Adjusting Azimuth ---\n"));

  if (targetAzimuth < azimuthDegMin || targetAzimuth > azimuthDegMax || targetElevation <= 0.0)
  {
    Log.println(F("[INFO] Solar azimuth is out of the tracking range or sun below the horizon. Cannot adjust azimuth."));

    if (currentAzimuth != 0.0)
    {
      Log.println(F("[INFO] Moving to the initial position."));
      moveFullLeft();

      if (isError)
      {
        Log.println(F("[ERROR] An error occurred during azimuth adjustment. "));
        Log.println(F("\n\t--- End of Azimuth Adjustment ---\n"));
        return -1.0;
      }
    }

    return currentAzimuth;
  }

  float azimuthDifference = fabs(targetAzimuth - currentAzimuth);
  float timeToMove = azimuthDifference / degreesPerMs; // Time in milliseconds

  if (timeToMove < azimuthTimeThreshold)
  {
    Log.println(F("[INFO] Time to move is less than the minimal required time. Cannot adjust azimuth."));
    return currentAzimuth;
  }

  Log.print(F("[INFO] Azimuth difference: "));
  Log.print(azimuthDifference, 2);
  Log.print(F("°. Time to move: "));
  Log.print(timeToMove);
  Log.println(F(" ms"));

  motorController->Enable();

  // Determine the direction of movement and move the motor
  if (targetAzimuth > currentAzimuth)
  {
    Log.println(F("[ADJUST] Moving motor to the right to align with the sun."));
    currentAzimuth += azimuthDifference;
    startMotorRight();
    if (waitForLimitSwitchOrDelay(timeToMove) == LIMIT_SWITCH_TRIGGERED)
    {
      Log.println(F("[WARNING] Limit switch was pressed during adjustment."));
      currentAzimuth = 360.0;
    }
  }
  else
  {
    Log.println(F("[ADJUST] Moving motor to the left to align with the sun."));
    currentAzimuth -= azimuthDifference;
    startMotorLeft();
    if (waitForLimitSwitchOrDelay(timeToMove) == LIMIT_SWITCH_TRIGGERED)
    {
      Log.println(F("[WARNING] Limit switch was pressed during adjustment."));
      currentAzimuth = 0.0;
    }
  }

  // Stop the motor after adjustment
  stopMotor();
  motorController->Disable();

  if (isError)
  {
    Log.println(F("[ERROR] An error occurred during azimuth adjustment. "));
    Log.println(F("\n\t--- End of Azimuth Adjustment ---\n"));
    return -1.0;
  }

  Log.print(F("[ADJUST] Azimuth aligned. Current azimuth: "));
  Log.println(currentAzimuth, 2);

  Log.println(F("\n\t--- End of Azimuth Adjustment ---\n"));

  return currentAzimuth;
}

// ----------------- Motor control functions -------------------

void AzimuthController::startMotorLeft()
{
  if (isError)
    return;

  Log.println(F("[MOTOR] Starting motor. Direction: left."));
  motorController->TurnLeft(motorPwmSpeed);
}

void AzimuthController::startMotorRight()
{
  if (isError)
    return;

  Log.println(F("[MOTOR] Starting motor. Direction: right."));
  motorController->TurnRight(motorPwmSpeed);
}

void AzimuthController::stopMotor()
{
  Log.println(F("[MOTOR] Stopping motor."));
  motorController->Stop();
}

// ----------------- Limit Switch control functions ------------------

/*
Function to wait for the limit switch to be pressed.
Returns:
- LIMIT_SWITCH_TRIGGERED: If the limit switch was pressed.
- ERROR_FULL_ROTATION_EXCEEDED: If the full rotation time was exceeded.
*/
int8_t AzimuthController::waitForLimitSwitch()
{
  return waitForLimitSwitchOrDelay(0);
}

/*
Function to wait for the limit switch to be pressed or a delay time to be reached.
Returns:
- LIMIT_SWITCH_TRIGGERED: If the limit switch was pressed.
- DELAY_TRIGGERED: If the delay time was reached.
- ERROR_FULL_ROTATION_EXCEEDED: If the full rotation time was exceeded.
*/
int8_t AzimuthController::waitForLimitSwitchOrDelay(uint32_t delayTime)
{
  uint32_t startTime = millis();
  while (true)
  {
    limitSwitch->loop();
    if (limitSwitch->isPressed())
    {
      return LIMIT_SWITCH_TRIGGERED;
    }
    if (delayTime > 0 && millis() - startTime >= delayTime)
    {
      return DELAY_TRIGGERED;
    }
    if (millis() - startTime >= fullRotationDuration * 1.05)
    {
      Log.println(F("[ERROR] Full rotation time exceeded. SHOULD NOT HAPPEN!."));
      isError = true;
      return ERROR_FULL_ROTATION_EXCEEDED;
    }
  }
}