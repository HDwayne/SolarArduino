#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include <Arduino.h>
#include <ezButton.h>

typedef void (*CommandCallback)();

struct joystickControllerConfig
{
  int8_t buttonPin;
  uint32_t buttonDebounce;
  int8_t vrxPin;
  int8_t vryPin;
  int16_t leftThreshold;
  int16_t rightThreshold;
  int16_t upThreshold;
  int16_t downThreshold;
};

class JoystickController
{
private:
  ezButton *button;
  int8_t vrxPin;
  int8_t vryPin;

  int16_t leftThreshold;
  int16_t rightThreshold;
  int16_t upThreshold;
  int16_t downThreshold;

  uint8_t prevCommand;

  CommandCallback onLeft;
  CommandCallback onRight;
  CommandCallback onUp;
  CommandCallback onDown;
  CommandCallback onLeaveLR;
  CommandCallback onLeaveUD;

  uint8_t getCommand();

public:
  JoystickController(const joystickControllerConfig &config);

  void setOnLeft(CommandCallback callback) { onLeft = callback; }
  void setOnRight(CommandCallback callback) { onRight = callback; }
  void setOnUp(CommandCallback callback) { onUp = callback; }
  void setOnDown(CommandCallback callback) { onDown = callback; }
  void setOnLeaveLR(CommandCallback callback) { onLeaveLR = callback; }
  void setOnLeaveUD(CommandCallback callback) { onLeaveUD = callback; }

  bool isPressed();
  void clearCommand();
  void executeCommand();
};

extern JoystickController joystickController;

#endif // JOYSTICK_CONTROLLER_H