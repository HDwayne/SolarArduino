#include "JoystickController.h"
#include "modules/ConfigModule.h"

enum Command
{
    COMMAND_NO = 0x00,
    COMMAND_LEFT = 0x01,
    COMMAND_RIGHT = 0x02,
    COMMAND_UP = 0x04,
    COMMAND_DOWN = 0x08
};

JoystickController joystickController;

// ----------------- Joystick Controller Constructor -----------------

JoystickController::JoystickController() {}

// ----------------- Public Methods -----------------

void JoystickController::init()
{
    button = new ezButton(configModule.getJoystickButtonPin());
    button->setDebounceTime(configModule.getJoystickButtonDebounce());
    vrxPin = configModule.getJoystickVrxPin();
    vryPin = configModule.getJoystickVryPin();

    pinMode(vrxPin, INPUT);
    pinMode(vryPin, INPUT);

    xCenter = analogRead(vrxPin);
    yCenter = analogRead(vryPin);

    uint8_t threshold = configModule.getJoystickThreshold();

    leftThreshold = xCenter - threshold;
    rightThreshold = xCenter + threshold;
    upThreshold = yCenter - threshold;
    downThreshold = yCenter + threshold;

    prevCommand = COMMAND_NO;

    onLeft = NULL;
    onRight = NULL;
    onUp = NULL;
    onDown = NULL;

    onLeaveLR = NULL;
    onLeaveUD = NULL;
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