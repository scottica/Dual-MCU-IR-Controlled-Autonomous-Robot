# Dual-MCU IR-Controlled Autonomous Robot

An embedded robotics system with a custom remote controller and an autonomous rover, written in bare-metal C. A **PIC32** microcontroller runs the remote and an **STM32** ARM Cortex-M0+ runs the robot. They communicate over a custom infrared protocol on a 38 kHz carrier. Built as a team project.

The robot follows the magnetic field of a guide wire, executes preset turns at intersections, stops for obstacles using a time-of-flight sensor, streams telemetry over Bluetooth Low Energy, and can be driven manually with a joystick.

## 🚀 Key Features

### 📡 Communication & PC Integration
* **Custom IR Protocol:** Each packet is a flag pulse, a ~100 ms gap, and a data pulse, with values encoded in pulse length. The PIC32 generates the 38 kHz carrier by toggling the IR LED from a 76 kHz timer interrupt, and the STM32 measures pulse lengths with `TIM21` and decodes them with a lookup table. Signal timing was measured with an oscilloscope to tune pulse widths against receiver noise.
* **BLE Telemetry & Dashboard:** An AT-09 BLE module streams coil readings, intersection count, and battery voltage to a Python dashboard (`bleak`), which can also select paths and upload custom routes from a `.csv` file.

### 🧠 Autonomous Navigation
* **Magnetic Field Sensing:** Three LC tank circuits tuned to the guide wire's signal, each with an LM358 amplifier and a peak detector feeding the STM32's ADC.
* **Line Following:** Compares the left and right coil readings and makes small corrective turns when their difference exceeds a threshold.
* **Intersection Routing:** A spike on the center coil marks an intersection, and a state machine (turning, intersection cooldown, driving) executes the preset or custom turn sequence.
* **Collision Stopping:** A VL53L0X time-of-flight sensor over I2C pauses the robot when an obstacle is within 100 mm. (The VL53L0X driver is adapted from an open-source library.)

### 🎮 Remote Controller
* **LCD Interface:** A 16x2 LCD shows the mode, a path progress bar, and an options menu for speed, path selection, reverse mode, and custom paths.
* **Joystick Control:** Joystick readings are calibrated at startup, converted to polar form (angle and magnitude), and sent over IR for manual driving.
* **Feedback:** Status LEDs and a speaker that beeps the selected path number.

### ⚙️ Mechanics
* **Claw:** A servo-driven claw closes automatically when an IR proximity sensor detects an object during a path, with manual control from the remote.

## 🧠 System Architecture

### Remote (PIC32MX130)
* 40 MHz MIPS32 core; joystick, pushbuttons with software debouncing, 16x2 LCD, speaker, LEDs, and IR LED driven by `Timer2` interrupts.

### Robot (STM32L051)
* 32 MHz ARM Cortex-M0+; IR receiver (`TIM21`), three inductor sensors (ADC), VL53L0X (I2C), IR proximity sensor, and AT-09 BLE module (USART2).
* Two gear motors driven by discrete MOSFET H-bridges, isolated from the logic by an LTV-847 optocoupler, with 100 Hz software PWM from `TIM2`. A servo drives the claw at 50 Hz.

## 💻 Tech Stack
* **Languages:** Bare-metal C, Python
* **Protocols:** I2C, UART, BLE, custom IR
* **Concepts:** Timer interrupts, software PWM, ADC sampling, state machines, analog signal conditioning

## ⚠️ Known Limitations
Turns are timed rather than measured, so turn accuracy varies with battery voltage and floor friction. Wheel encoders or a gyroscope would allow closed-loop turns.
