#ifndef AZIMUTH_CONTROLLER_H
#define AZIMUTH_CONTROLLER_H

#include <Arduino.h>
#include <ezButton.h>
#include <BTS7960.h>

class AzimuthController
{
public:
  // Constructor
  AzimuthController(int motorEnPin, int motorPwmLeftPin, int motorPwmRightPin, int motorSpeed, int limitSwitchPin, float maxAzimuth, float minAzimuth, float degThreshold, unsigned long timeThreshold);

  // Calibration and movement methods
  void calibrate();
  void moveFullLeft();
  void moveFullRight();
  void moveToAngle(float targetAngle);

private:
  // Pin Definitions
  int motorPinEn;     // Motor enable pin
  int motorPinPwmL;   // Motor PWM pin (left)
  int motorPinPwmR;   // Motor PWM pin (right)
  int limitSwitchPin; // Limit switch pin

  // Motor Settings
  int motorPwmSpeed; // Motor PWM speed

  // Motor Controller and Limit Switch
  BTS7960 motorController;
  ezButton limitSwitch;

  // Variables
  unsigned long fullRotationDuration; // Duration of a full rotation in milliseconds
  float currentAzimuth;               // Current azimuth position of the solar panel
  float degreesPerMs;                 // Degrees the motor turns per millisecond

  // Constants
  const float azimuthDegMax;
  const float azimuthDegMin;
  const float azimuthDegThreshold;
  const unsigned long azimuthTimeThreshold;

  // Utility methods
  void startMotorLeft();
  void startMotorRight();
  void stopMotor();
  void waitForLimitSwitch();
  bool waitForLimitSwitchOrDelay(unsigned long delayTime);
};

#endif // AZIMUTH_CONTROLLER_H
