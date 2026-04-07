# Micromouse firmware

This repository hosts the STM32CubeIDE firmware for a micromouse running on an STM32F401CCU6-based board. The code balances Cube-generated peripherals with bespoke control and sensor modules so the robot can run velocity, heading, and wall-following behaviors under a single control loop.

## Structure

- `micromouse/`: STM32CubeIDE project; `Core/` holds startup, HAL configuration, and `main.c`, while the rest of the folder contains the bespoke control, sensor, and utility modules.
- `micromouse/Drivers/VL53L0X/`: official ST drivers for the dual VL53L0X time-of-flight sensors on I2C1 and I2C2.
- `Test Projects/`: archival or experimental branches of the firmware that are not required for the current build.

## What the firmware covers

- Peripheral initialization for GPIO, dual I2C, UART2, TIM1/TIM4 encoders, TIM2 PWM (two channels), and TIM10-driven control timing.
- Sensor bring-up, calibration, and measurement for both VL53L0X modules alongside encoder feedback to monitor wheel velocities.
- Control loop logic that generates translational/angular requests, applies PID regulators, and toggles wall-following or heading correction modes as needed.

## Building and flashing

1. Open `micromouse/micromouse.ioc` in STM32CubeIDE.
2. Inspect or regenerate peripheral settings if you update the `.ioc`; CubeIDE will keep the `Core/` and `Drivers/` folders aligned.
3. Build and flash through STM32CubeIDE (ST-Link). There is no automated CLI build in this repo.

## Reviewer notes

- Focus on `micromouse/Core/Src/main.c` and the custom control/sensor modules when following the control path; most other folders are Cube-generated or legacy.
- The other debug launch file (`micromouse Debug.launch`) already targets the renamed project; it is sufficient for standard debugging workflows.
- This README keeps the story tight so GSoC reviewers can judge capabilities, structure, and build steps with minimal scanning.

## License

See `LICENSE`.
