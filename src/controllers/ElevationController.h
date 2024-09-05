#ifndef ELEVATION_CONTROLLER_H
#define ELEVATION_CONTROLLER_H

#include <Arduino.h>
#include <BTS7960.h>

struct ElevationControllerConfig
{
  int motorEnPin;
  int motorPwmUpPin;
  int motorPwmDownPin;
  int motorPwmSpeed;
  float maxAzimuth;
  float minAzimuth;
  float maxElevation;
  float minElevation;
  unsigned long timeThreshold;
  float actuatorSpeed;
  float actuatorLength;
};

class ElevationController
{
public:
  // Constructor
  ElevationController(const ElevationControllerConfig &config);

  // Movement methods
  void calibrate();
  void moveToAngle(float targetAzimuth, float targetElevation);
  void moveToMaxElevation();
  void moveToMinElevation();
  void startActuatorUp();
  void startActuatorDown();
  void stopActuator();
  void enableMotor() { motorController.Enable(); }
  void disableMotor() { motorController.Disable(); }

private:
  // Pin Definitions
  int motorPinEn;      // Motor enable pin
  int motorPinPwmUp;   // Motor PWM pin for actuator extension (up)
  int motorPinPwmDown; // Motor PWM pin for actuator retraction (down)
  int motorPwmSpeed;   // Motor PWM speed

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
  const float azimuthDegMax;
  const float azimuthDegMin;
  const float elevationDegMax;
  const float elevationDegMin;
  const unsigned long elevationTimeThreshold;
};

extern ElevationController elevationController;

#endif // ELEVATION_CONTROLLER_H
