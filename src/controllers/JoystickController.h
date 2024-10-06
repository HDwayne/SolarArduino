#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include <Arduino.h>
#include <ezButton.h>

typedef void (*CommandCallback)();

class JoystickController
{
private:
  ezButton *button;
  uint8_t vrxPin;
  uint8_t vryPin;

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
  JoystickController();
  void init();

  void setOnLeft(CommandCallback callback) { onLeft = callback; }
  void setOnRight(CommandCallback callback) { onRight = callback; }
  void setOnUp(CommandCallback callback) { onUp = callback; }
  void setOnDown(CommandCallback callback) { onDown = callback; }
  void setOnLeaveLR(CommandCallback callback) { onLeaveLR = callback; }
  void setOnLeaveUD(CommandCallback callback) { onLeaveUD = callback; }

  bool isPressed();
  void clearCommand();
  void executeCommand();

private:
  int xCenter = 0;
  int yCenter = 0;
};

extern JoystickController joystickController;

#endif // JOYSTICK_CONTROLLER_H