#ifndef ELEVATION_CONTROLLER_H
#define ELEVATION_CONTROLLER_H

#include <Arduino.h>
#include <BTS7960.h>

class ElevationController
{
public:
  // Constructor
  ElevationController();

  // Movement methods
  void init();
  float moveToAngle(float targetAzimuth, float targetElevation);
  void moveToMaxElevation();
  void moveToMinElevation();
  void startActuatorUp();
  void startActuatorDown();
  void stopActuator();
  void enableMotor() { motorController->Enable(); }
  void disableMotor() { motorController->Disable(); }
  float getCurrentElevation() { return currentElevation; }

private:
  // Pin Definitions
  uint8_t motorPinEn;      // Motor enable pin
  uint8_t motorPinPwmUp;   // Motor PWM pin for actuator extension (up)
  uint8_t motorPinPwmDown; // Motor PWM pin for actuator retraction (down)
  uint8_t motorPwmSpeed;   // Motor PWM speed

  // Motor Controller
  BTS7960 *motorController;

  // Variables
  float currentElevation; // Current elevation position of the solar panel

  // Actuator parameters
  float actuatorSpeed;             // Speed of the actuator in mm/s
  float actuatorLength;            // Length of the actuator in mm
  uint32_t actuatorFullTravelTime; // Time in milliseconds to move the actuator from fully retracted to fully extended
  float degreesPerMs;              // Degrees the actuator moves per millisecond

  // Constants
  float azimuthDegMax;
  float azimuthDegMin;
  float elevationDegMax;
  float elevationDegMin;
  uint32_t elevationTimeThreshold;
};

#endif // ELEVATION_CONTROLLER_H
