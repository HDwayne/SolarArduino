#ifndef AZIMUTH_CONTROLLER_H
#define AZIMUTH_CONTROLLER_H

#include <Arduino.h>
#include <ezButton.h>
#include <BTS7960.h>

struct AzimuthControllerConfig
{
  uint8_t motorEnPin;
  uint8_t motorPwmLeftPin;
  uint8_t motorPwmRightPin;
  uint8_t motorSpeed;
  uint8_t limitSwitchPin;
  float maxAzimuth;
  float minAzimuth;
  uint32_t timeThreshold;
};

class AzimuthController
{
public:
  // Constructor
  AzimuthController(const AzimuthControllerConfig &config);

  // Calibration and movement methods
  void calibrate();
  void moveFullLeft();
  void moveFullRight();
  void moveToAngle(float targetAngle);
  void startMotorLeft();
  void startMotorRight();
  void stopMotor();
  void enableMotor() { motorController.Enable(); }
  void disableMotor() { motorController.Disable(); }

private:
  // Pin Definitions
  uint8_t motorPinEn;     // Motor enable pin
  uint8_t motorPinPwmL;   // Motor PWM pin (left)
  uint8_t motorPinPwmR;   // Motor PWM pin (right)
  uint8_t motorPwmSpeed;  // Motor PWM speed
  uint8_t limitSwitchPin; // Limit switch pin

  // Motor Controller and Limit Switch
  BTS7960 motorController;
  ezButton limitSwitch;

  // Variables
  uint32_t fullRotationDuration; // Duration of a full rotation in milliseconds
  float currentAzimuth;          // Current azimuth position of the solar panel
  float degreesPerMs;            // Degrees the motor turns per millisecond

  // Constants
  const float azimuthDegMax;
  const float azimuthDegMin;
  const uint32_t azimuthTimeThreshold;

  // Utility methods
  void waitForLimitSwitch();
  bool waitForLimitSwitchOrDelay(uint32_t delayTime);
};

extern AzimuthController azimuthController;

#endif // AZIMUTH_CONTROLLER_H
