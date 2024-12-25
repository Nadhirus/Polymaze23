# Polymaze23 Line-Following Robot

This repository contains the firmware for a line-following robot developed for participation in **Polymaze23**. The robot uses an ESP32 DevBoard, programmed via the PlatformIO IDE with the Arduino framework. The code implements motion control, PID algorithms, and a basic serial interface for configuration.

## Features

### 1. **Motion Control**
   - **Differential Drive Kinematics**: Computes left and right wheel velocities based on linear and angular velocity commands.
   - **PWM Motor Control**: Writes PWM signals to the motors for precise speed adjustments.

### 2. **PID Speed Regulation**
   - Individual PID controllers for each motor ensure the robot maintains the desired wheel speeds.
   - Configurable PID constants via serial interface.

### 3. **Encoder Feedback**
   - Real-time wheel speed calculation using quadrature encoders.
   - Direction detection based on encoder signals.

### 4. **Serial Communication Interface**
   - Configures PID parameters (`Kp`, `Ki`, `Kd`) and reference speeds via a simple serial protocol.
   - Reports wheel speeds and reference values for debugging.

## Pin Definitions
- **Encoders**: `R_CHA`, `R_CHB`, `L_CHA`, `L_CHB`
- **Motors**: `R_MOTORA`, `R_MOTORB`, `L_MOTORA`, `L_MOTORB`

Ensure these pins are correctly defined in `PinDefinitions.h` to match your hardware configuration.

## Installation

1. Install [PlatformIO](https://platformio.org/).
2. Clone this repository:
   ```bash
   git clone https://github.com/nadhirus/polymaze23.git
  	```

3. Open the project in PlatformIO.  
4. Connect the ESP32 DevBoard to your computer and upload the firmware:

    ```bash
    pio run --target upload
    ```

## Usage

1. **Startup**:
   - Power on the ESP32 DevBoard and attach the robot to the maze.

2. **Configuration**:
   - Use a serial terminal (e.g., PlatformIO Serial Monitor) to send PID parameters and reference speeds.
   - Parameters should be sent in the following order (as `float` values):
     ```text
     Kp_right Ki_right Kd_right RefSpeed_right Kp_left Ki_left Kd_left RefSpeed_left
     ```

3. **Operation**:
   - The robot continuously adjusts motor speeds based on PID feedback.
   - Outputs wheel speed and reference data to the serial terminal for real-time monitoring.

## Code Structure

- **`setup()`**: Initializes pins, interrupts, and serial communication.
- **`loop()`**: Main control loop for speed calculation, PID updates, and motor control.
- **Key Functions**:
  - `Kinematics()`: Converts velocity commands to wheel RPMs.
  - `motor_write()`: Sets motor PWM signals.
  - `PID_right()` / `PID_left()`: PID control logic for the right and left wheels.
  - `right_wheel_pulse()` / `left_wheel_pulse()`: Encoder pulse handling.
  - `report()`: Sends speed data to the serial terminal.
  - `interface()`: Parses serial commands.

## Contributing

Contributions are welcome! Please open an issue or pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
