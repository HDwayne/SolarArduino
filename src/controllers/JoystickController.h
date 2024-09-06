#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include <Arduino.h>
#include <ezButton.h>

typedef void (*CommandCallback)();

enum Command
{
  COMMAND_NO = 0x00,
  COMMAND_LEFT = 0x01,
  COMMAND_RIGHT = 0x02,
  COMMAND_UP = 0x04,
  COMMAND_DOWN = 0x08
};

struct joystickControllerConfig
{
  uint8_t buttonPin;
  uint8_t vrxPin;
  uint8_t vryPin;
  int16_t leftThreshold;
  int16_t rightThreshold;
  int16_t upThreshold;
  int16_t downThreshold;
};

class JoystickController
{
private:
  ezButton *button; // Button for joystick press
  uint8_t vrxPin;   // Pin for horizontal axis
  uint8_t vryPin;   // Pin for vertical axis

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

  uint8_t getCommand()
  {
    int xValue = analogRead(vrxPin);
    int yValue = analogRead(vryPin);

    uint8_t command = COMMAND_NO;

    if (xValue < leftThreshold)
    {
      command |= COMMAND_LEFT;
    }
    else if (xValue > rightThreshold)
    {
      command |= COMMAND_RIGHT;
    }

    if (yValue < upThreshold)
    {
      command |= COMMAND_UP;
    }
    else if (yValue > downThreshold)
    {
      command |= COMMAND_DOWN;
    }

    return command;
  }

  bool commandChanged(uint8_t newCommand)
  {
    if (newCommand != prevCommand)
    {
      return true;
    }
    return false;
  }

public:
  JoystickController(const joystickControllerConfig &config)
  {
    button = new ezButton(config.buttonPin);
    button->setDebounceTime(5000);
    vrxPin = config.vrxPin;
    vryPin = config.vryPin;
    leftThreshold = config.leftThreshold;
    rightThreshold = config.rightThreshold;
    upThreshold = config.upThreshold;
    downThreshold = config.downThreshold;
    prevCommand = COMMAND_NO;
  }

  void setOnLeft(CommandCallback callback) { onLeft = callback; }
  void setOnRight(CommandCallback callback) { onRight = callback; }
  void setOnUp(CommandCallback callback) { onUp = callback; }
  void setOnDown(CommandCallback callback) { onDown = callback; }
  void setOnLeaveLR(CommandCallback callback) { onLeaveLR = callback; }
  void setOnLeaveUD(CommandCallback callback) { onLeaveUD = callback; }

  bool isPressed()
  {
    button->loop();
    return button->isPressed();
  }

  void clearCommand()
  {
    prevCommand = COMMAND_NO;
  }

  void executeCommand()
  {
    uint8_t command = getCommand();

    if (commandChanged(command))
    {
      if ((prevCommand & (COMMAND_LEFT | COMMAND_RIGHT)) != (command & (COMMAND_LEFT | COMMAND_RIGHT)))
      {
        onLeaveLR();
      }
      if ((prevCommand & (COMMAND_UP | COMMAND_DOWN)) != (command & (COMMAND_UP | COMMAND_DOWN)))
      {
        onLeaveUD();
      }

      if (onLeft && (command & COMMAND_LEFT))
      {
        onLeft();
      }
      else if (onRight && (command & COMMAND_RIGHT))
      {
        onRight();
      }

      if (onUp && (command & COMMAND_UP))
      {
        onUp();
      }
      else if (onDown && (command & COMMAND_DOWN))
      {
        onDown();
      }

      prevCommand = command;
    }
  }
};

extern JoystickController joystickController;

#endif // JOYSTICK_CONTROLLER_H