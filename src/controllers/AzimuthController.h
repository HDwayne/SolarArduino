#ifndef AZIMUTH_CONTROLLER_H
#define AZIMUTH_CONTROLLER_H

#include <Arduino.h>
#include <ezButton.h>
#include <BTS7960.h>

#define DELAY_TRIGGERED 0
#define LIMIT_SWITCH_TRIGGERED 1
#define ERROR_FULL_ROTATION_EXCEEDED -1

class AzimuthController
{
public:
  // Constructor
  AzimuthController();
  int8_t init();

  // Calibration and movement methods
  int8_t moveFullLeft();
  int8_t moveFullRight();
  float moveToAngle(float targetAzimuth, float targetElevation);
  void startMotorLeft();
  void startMotorRight();
  void stopMotor();
  void enableMotor() { motorController->Enable(); }
  void disableMotor() { motorController->Disable(); }
  float getCurrentAzimuth() { return currentAzimuth; }

private:
  // Pin Definitions
  uint8_t motorPinEn;     // Motor enable pin
  uint8_t motorPinPwmL;   // Motor PWM pin (left)
  uint8_t motorPinPwmR;   // Motor PWM pin (right)
  uint8_t motorPwmSpeed;  // Motor PWM speed
  uint8_t limitSwitchPin; // Limit switch pin

  // Motor Controller and Limit Switch
  BTS7960 *motorController;
  ezButton *limitSwitch;

  // Variables
  uint32_t fullRotationDuration; // Duration of a full rotation in milliseconds
  float currentAzimuth;          // Current azimuth position of the solar panel
  float degreesPerMs;            // Degrees the motor turns per millisecond

  // Constants
  float azimuthDegMax;
  float azimuthDegMin;
  uint32_t azimuthTimeThreshold;

  bool isError = false;

  // Utility methods
  int8_t waitForLimitSwitch();
  int8_t waitForLimitSwitchOrDelay(uint32_t delayTime);
};

extern AzimuthController azimuthController;

#endif // AZIMUTH_CONTROLLER_H
