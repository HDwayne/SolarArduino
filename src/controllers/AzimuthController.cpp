#include "AzimuthController.h"

AzimuthController::AzimuthController(const AzimuthControllerConfig &config)
    : motorPinEn(config.motorEnPin),
      motorPinPwmL(config.motorPwmLeftPin),
      motorPinPwmR(config.motorPwmRightPin),
      limitSwitchPin(config.limitSwitchPin),
      motorPwmSpeed(config.motorSpeed),
      motorController(config.motorEnPin, config.motorPwmLeftPin, config.motorPwmRightPin),
      limitSwitch(config.limitSwitchPin),
      fullRotationDuration(0),
      currentAzimuth(0.0),
      degreesPerMs(0.0),
      azimuthDegMax(config.maxAzimuth),
      azimuthDegMin(config.minAzimuth),
      azimuthTimeThreshold(config.timeThreshold)
{
  limitSwitch.setDebounceTime(100);
}

// ----------------- Azimuth control functions -----------------

void AzimuthController::calibrate()
{
  Serial.println(F("\n\t--- Starting Calibration Procedure ---\n"));

  moveFullRight();
  delay(3000);

  unsigned long pressStartTime = millis();
  moveFullLeft();

  fullRotationDuration = millis() - pressStartTime;
  Serial.print(F("\n\tCalibration Duration: "));
  Serial.print(fullRotationDuration);
  Serial.println(F(" ms"));

  // Calculate degrees per millisecond
  degreesPerMs = 360.0 / fullRotationDuration;
  Serial.print(F("\tDegrees per millisecond: "));
  Serial.print(degreesPerMs, 6);
  Serial.println(F("°/ms\n"));

  Serial.println(F("\n\t--- Calibration procedure completed. ---\n"));
}

void AzimuthController::moveFullLeft()
{
  Serial.println(F("-> Moving to full left position"));
  motorController.Enable();

  startMotorLeft();
  waitForLimitSwitch();
  stopMotor();

  motorController.Disable();

  currentAzimuth = 0.0;
  Serial.println(F("-> Full left position reached"));
}

void AzimuthController::moveFullRight()
{
  Serial.println(F("-> Moving to full right position"));
  motorController.Enable();

  startMotorRight();
  waitForLimitSwitch();
  stopMotor();

  motorController.Disable();

  currentAzimuth = 360.0;
  Serial.println(F("-> Full right position reached"));
}

void AzimuthController::moveToAngle(float targetAngle)
{
  Serial.println(F("\n\t--- Adjusting Azimuth ---\n"));

  if (targetAngle < azimuthDegMin || targetAngle > azimuthDegMax)
  {
    Serial.println(F("[INFO] Solar azimuth is out of the tracking range. Cannot adjust azimuth."));

    if (currentAzimuth != 0.0)
    {
      Serial.println(F("[INFO] Moving to the initial position."));
      moveFullLeft();
    }

    return;
  }

  float azimuthDifference = fabs(targetAngle - currentAzimuth);
  float timeToMove = azimuthDifference / degreesPerMs; // Time in milliseconds

  if (timeToMove < azimuthTimeThreshold)
  {
    Serial.println(F("[INFO] Time to move is less than the minimal required time. Cannot adjust azimuth."));
    return;
  }

  Serial.print(F("[INFO] Azimuth difference: "));
  Serial.print(azimuthDifference, 2);
  Serial.print(F("°. Time to move: "));
  Serial.print(timeToMove);
  Serial.println(F(" ms"));

  motorController.Enable();

  // Determine the direction of movement and move the motor
  if (targetAngle > currentAzimuth)
  {
    Serial.println(F("[ADJUST] Moving motor to the right to align with the sun."));
    currentAzimuth += azimuthDifference;
    startMotorRight();
    if (waitForLimitSwitchOrDelay(timeToMove))
    {
      Serial.println(F("[WARNING] Limit switch was pressed during adjustment."));
      currentAzimuth = 360.0;
    }
  }
  else
  {
    Serial.println(F("[ADJUST] Moving motor to the left to align with the sun."));
    currentAzimuth -= azimuthDifference;
    startMotorLeft();
    if (waitForLimitSwitchOrDelay(timeToMove))
    {
      Serial.println(F("[WARNING] Limit switch was pressed during adjustment."));
      currentAzimuth = 0.0;
    }
  }

  // Stop the motor after adjustment
  stopMotor();
  motorController.Disable();

  Serial.print(F("[ADJUST] Azimuth aligned. Current azimuth: "));
  Serial.println(currentAzimuth, 2);

  Serial.println(F("\n\t--- End of Azimuth Adjustment ---\n"));
}

// ----------------- Motor control functions -------------------

void AzimuthController::startMotorLeft()
{
  Serial.println(F("[MOTOR] Starting motor. Direction: left."));
  motorController.TurnLeft(motorPwmSpeed);
}

void AzimuthController::startMotorRight()
{
  Serial.println(F("[MOTOR] Starting motor. Direction: right."));
  motorController.TurnRight(motorPwmSpeed);
}

void AzimuthController::stopMotor()
{
  Serial.println(F("[MOTOR] Stopping motor."));
  motorController.Stop();
}

// ----------------- Limit Switch control functions ------------------

// function blocks until the Limit Switch is reached
void AzimuthController::waitForLimitSwitch()
{
  while (true)
  {
    limitSwitch.loop();
    if (limitSwitch.isPressed())
    {
      Serial.println(F("[SWITCH] Limit switch pressed."));
      break;
    }
  }
  limitSwitch.loop();
}

// function blocks until the Limit Switch is reached or the delay time has passed. Returns true if the Limit Switch was pressed.
bool AzimuthController::waitForLimitSwitchOrDelay(unsigned long delayTime)
{
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime)
  {
    limitSwitch.loop();
    if (limitSwitch.isPressed())
    {
      Serial.println(F("[SWITCH] Limit switch pressed."));
      return true;
    }
  }
  limitSwitch.loop();
  return false;
}