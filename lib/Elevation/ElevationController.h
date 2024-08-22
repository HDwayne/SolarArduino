#ifndef ELEVATION_CONTROLLER_H
#define ELEVATION_CONTROLLER_H

#include <Arduino.h>
#include <BTS7960.h>

#define ELEVATION_MOTOR_PWM_SPEED 255 // Maximum PWM speed for the motor driver

class ElevationController
{
public:
  // Constructor
  ElevationController(int motorEnPin, int motorPwmUpPin, int motorPwmDownPin, float maxElevation, float minElevation, unsigned long timeThreshold, float actuatorSpeed, float actuatorLength);

  // Movement methods
  void calibrate();
  void moveToAngle(float targetAngle);
  void moveToMaxElevation();
  void moveToMinElevation();

private:
  // Pin Definitions
  int motorPinEn;      // Motor enable pin
  int motorPinPwmUp;   // Motor PWM pin for actuator extension (up)
  int motorPinPwmDown; // Motor PWM pin for actuator retraction (down)

  // Motor Controller
  BTS7960 motorController;

  // Variables
  float currentElevation; // Current elevation position of the solar panel

  // Actuator parameters
  float actuatorSpeed;                  // Speed of the actuator in mm/s
  float actuatorLength;                 // Length of the actuator in mm
  unsigned long actuatorFullTravelTime; // Time in milliseconds to move the actuator from fully retracted to fully extended
  float degreesPerMs;                   // Degrees the actuator moves per millisecond

  // Constants
  const float elevationDegMax;
  const float elevationDegMin;
  const unsigned long elevationTimeThreshold;

  // Utility methods
  void startActuatorUp();
  void startActuatorDown();
  void stopActuator();
};

#endif // ELEVATION_CONTROLLER_H
