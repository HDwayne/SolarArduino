#include "JoystickController.h"
#include "config.h"

joystickControllerConfig joystickConfig = {
    JOYSTICK_BUTTON_PIN,
    JOYSTICK_VRX_PIN,
    JOYSTICK_VRY_PIN,
    JOYSTICK_LEFT_THRESHOLD,
    JOYSTICK_RIGHT_THRESHOLD,
    JOYSTICK_UP_THRESHOLD,
    JOYSTICK_DOWN_THRESHOLD};

JoystickController joystickController(joystickConfig);