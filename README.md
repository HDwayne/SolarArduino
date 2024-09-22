![Solar Tracker image](/img/solar_tracker_main.JPG "Solar Tracker")

# Solar tracker with ARDUINO or ESP32 and soltrack-2.2 library

This Solar Tracker is an embedded system that uses an Arduino or ESP32 microcontroller to track the sun's position and adjust the angle of a solar panel accordingly. By tracking the sun’s movement throughout the day, the Solar Tracker ensures the solar panel is always optimally positioned for maximum energy production.

The system computes the sun’s position using the [SolTrack 2.2](https://github.com/MarcvdSluys/SolTrack) library. This project supports two microcontrollers: **Arduino** and **ESP32**.

## Features

| **Feature**                           | **Arduino**           | **ESP32** |
|---------------------------------------|-----------------------|-----------|
| **Microcontroller**                   | Arduino UNO/MEGA2560  | ESP32     |
| **Time Management (RTC)**             | Manual adjustment of date and time | Automatic adjustment via Wi-Fi |
| **Wi-Fi Connectivity**                | Not available         | Available |
| **MQTT Support**                      | Not available         | Fully supported for Home Assistant |
| **Motor Control (Azimuth/Elevation)** | Supported             | Supported |
| **Joystick Control**                  | Supported             | Supported |
| **Anemometer**                        | Supported             | Supported |
| **Solar Position Calculation**        | Supported             | Supported |

# how it works

### Azimuth Control Overview

![Azimuth control](/img/azimuth.JPG "Azimuth control")

The **azimuth controller** manages the solar panel's horizontal movement, powered by a motor controlled via a **BTS7960** driver. A **southward limit switch** acts as a reference to stop the motor when the panel reaches the defined position, preventing excessive rotation.

#### Pin Configurations

| **Component**         | **Arduino**           | **ESP32**          |
|-----------------------|-----------------------|--------------------|
| Motor Enable Pin       | Pin 4                 | GPIO 19            |
| PWM Left (Move Left)   | Pin 6                 | GPIO 18            |
| PWM Right (Move Right) | Pin 5                 | GPIO 17            |
| Limit Switch           | Pin 7                 | GPIO 16            |

#### Calibration and Movement

At startup, the solar tracker goes through a **calibration process** to measure how long it takes for the panel to rotate fully from one side to the other (from 0° to 360° azimuth). Once calibrated, the tracker is able to adjust the azimuth angle by calculating the time needed to reach the desired position. The system also checks if the target azimuth angle is within the allowed range (`AZIMUTH_DEG_MIN` to `AZIMUTH_DEG_MAX`). If outside, the movement is canceled.

#### Limit Switch

The **limit switch** ensures the panel doesn’t exceed its bounds, defining 0° (or 360°) when pressed. If triggered during operation, the motor stops immediately. It also acts as a reference point to determine the panel's azimuth angle.

Here is a summary of the azimuth control parameters, as defined in `config.h`:

| **Parameter**                    | **Value**                  |
|---------------------------------- |----------------------------|
| **AZIMUTH_MOTOR_PIN_EN**          | Motor enable pin            |
| **AZIMUTH_MOTOR_PWM_PIN_L**       | Motor PWM pin for left      |
| **AZIMUTH_MOTOR_PWM_PIN_R**       | Motor PWM pin for right     |
| **AZIMUTH_LIMIT_SWITCH_PIN**      | Limit switch pin            |
| **AZIMUTH_DEG_MAX**               | 275.0° (maximum azimuth allowed) |
| **AZIMUTH_DEG_MIN**               | 90.0° (minimum azimuth allowed) |
| **AZIMUTH_TIME_THRESHOLD**        | Minimum time threshold (optional) |

### Elevation Control Overview

![Elevation control](/img/elevation.JPG "Elevation control")

The **elevation controller** manages the solar panel’s vertical tilt, tracking the sun's altitude. This is done via an actuator controlled by a **BTS7960** motor driver.

> **Note:** Relay or H-Bridge could work, I used a BTS7960 cause i had one laying around.

#### Pin Configurations for Arduino and ESP32

| **Component**         | **Arduino**           | **ESP32**          |
|-----------------------|-----------------------|--------------------|
| Motor Enable Pin       | Pin 8                 | GPIO 26            |
| PWM Up (Extend)        | Pin 10                | GPIO 27            |
| PWM Down (Retract)     | Pin 9                 | GPIO 14            |

#### Calibration and Movement

To ensure correct actuator operation, use the datasheet to configure `ELEVATION_ACTUATOR_SPEED` and `ELEVATION_ACTUATOR_LENGTH`. These parameters calculate the time required to fully extend or retract the actuator. If real-world conditions vary, override this time using the `FORCE_TIME_FULL_TRAVEL` parameter.

> **Warning:** Keep the PWM speed at 255 (maximum) for accurate time calculations during extension and retraction.

At initialization, the system calibrates the actuator, setting the panel to a 90° (horizontal) position for safety. The elevation range is constrained by the actuator’s length and physical limits of the panel. Customize `ELEVATION_DEG_MIN` and `ELEVATION_DEG_MAX` based on your setup.

> I used SolidWorks to simulate the panel’s movement and constraints.

#### Key Parameters in `config.h`

| **Parameter**                    | **Value**                  |
|---------------------------------- |----------------------------|
| **ELEVATION_MOTOR_PIN_EN**        | Motor enable pin            |
| **ELEVATION_MOTOR_PWM_PIN_U**     | PWM pin for extension (up)  |
| **ELEVATION_MOTOR_PWM_PIN_D**     | PWM pin for retraction (down) |
| **ELEVATION_DEG_MAX**             | 90.0°                       |
| **ELEVATION_DEG_MIN**             | 19.0°                       |
| **ELEVATION_MOTOR_PWM_SPEED**     | 255 (maximum motor speed)   |
| **ELEVATION_ACTUATOR_SPEED**      | 5 mm/s                      |
| **ELEVATION_ACTUATOR_LENGTH**     | 350 mm                      |
| **FORCE_TIME_FULL_TRAVEL**        | Forced full travel time (in seconds) |

### Joystick Control

Joystick control is available for both azimuth and elevation adjustments. The system reads the joystick’s direction and adjusts the panel’s position accordingly.
It could be useful for manual adjustments or testing.

### Anemometer




# Special thanks to the SolTrack library

Thanks to the [soltrack-2.2 library](https://github.com/MarcvdSluys/SolTrack). This library is used to calculate the sun's position based on the date, time, and location. The library provides the azimuth and elevation angles of the sun, which are used to adjust the solar panel's position.



