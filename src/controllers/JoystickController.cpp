#include "JoystickController.h"
#include "config.h"

enum Command
{
    COMMAND_NO = 0x00,
    COMMAND_LEFT = 0x01,
    COMMAND_RIGHT = 0x02,
    COMMAND_UP = 0x04,
    COMMAND_DOWN = 0x08
};

joystickControllerConfig joystickConfig = {
    JOYSTICK_BUTTON_PIN,
    JOYSTICK_BUTTON_DEBOUNCE,
    JOYSTICK_VRX_PIN,
    JOYSTICK_VRY_PIN,
    JOYSTICK_THRESHOLD};

JoystickController joystickController(joystickConfig);

// ----------------- Joystick Controller Constructor -----------------

JoystickController::JoystickController(const joystickControllerConfig &config)
{
    button = new ezButton(config.buttonPin);
    button->setDebounceTime(config.buttonDebounce);
    vrxPin = config.vrxPin;
    vryPin = config.vryPin;
    prevCommand = COMMAND_NO;

    xCenter = analogRead(vrxPin);
    yCenter = analogRead(vryPin);

    leftThreshold = xCenter - config.threshold;
    rightThreshold = xCenter + config.threshold;
    upThreshold = yCenter - config.threshold;
    downThreshold = yCenter + config.threshold;
}

// ---------------- Private Methods ----------------

uint8_t JoystickController::getCommand()
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

// ---------------- Public Methods ----------------

bool JoystickController::isPressed()
{
    button->loop();
    return button->isPressed();
}

void JoystickController::clearCommand()
{
    prevCommand = COMMAND_NO;
}

void JoystickController::executeCommand()
{
    uint8_t command = getCommand();

    if (command != prevCommand)
    {
        if (onLeaveLR && ((prevCommand & (COMMAND_LEFT | COMMAND_RIGHT)) != (command & (COMMAND_LEFT | COMMAND_RIGHT))))
        {
            onLeaveLR();
        }
        if (onLeaveUD && ((prevCommand & (COMMAND_UP | COMMAND_DOWN)) != (command & (COMMAND_UP | COMMAND_DOWN))))
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