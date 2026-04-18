# Dual-MCU IR-Controlled Autonomous Robot

An end-to-end embedded robotics system featuring a custom remote controller and an autonomous rover. Built from the ground up using bare-metal C, the system utilizes a **PIC32** microcontroller as a smart remote and an **STM32** ARM Cortex-M0+ as the robot's brain, communicating entirely via a custom-designed 76kHz Infrared (IR) protocol.

The robot is capable of autonomous magnetic line-following, intelligent intersection navigation, Time-of-Flight (ToF) obstacle avoidance, two-way Bluetooth telemetry, and remote-controlled manual override using an analog joystick.

## 🚀 Key Features

### 📡 Telemetry & PC Integration
* **Two-Way Bluetooth Communication:** Streams real-time robot statistics (including live battery percentage and current location/progress on the path) back to the user, while allowing remote path selection.
* **CSV Custom Routing:** Supports loading custom navigation paths dynamically from a PC via Bluetooth by parsing a `.csv` file.
* **Custom IR Protocol:** Designed a robust 76kHz IR protocol using pulse-count modulation to encode joystick vectors and commands, decoded on the receiver via hardware timers.

### 🧠 Autonomous Navigation
* **Sensor Fusion & Magnetic Tracking:** Fuses data from three differential analog magnetic coils via ADC to track lines and detect intersections. 
* **Dynamic Intersection Routing:** A programmable state machine executes predefined, custom, or auto-reversing turn sequences at intersections.
* **ToF Obstacle Avoidance:** Integrates a VL53L0X Time-of-Flight laser sensor via I2C to detect obstacles within 100mm, automatically pausing motor execution to prevent collisions.

### 🎮 Smart Remote & UI
* **Interactive LCD Dashboard:** The PIC32 remote features a 16x2 LCD displaying a dynamic real-time progress bar for the robot's path, battery stats, and an Options Menu to configure autonomous PWM base speeds.
* **Proportional Manual Control:** The analog joystick calculates Euclidean vectors to provide smooth, proportional PWM speed control during manual override.
* **Multi-Sensory Feedback:** Includes distinct LED indicators (Blue for active navigation, solid Green for idle, and flashing Green upon destination arrival) and audio feedback, with the controller speaker beeping to indicate the active path number.

### ⚙️ Automated Mechanics
* **Smart Claw System:** A PWM-driven servo claw that automatically detects and grips objects using a dedicated IR proximity sensor, holding them securely until automatically dropping them off at the final destination. Supports full manual override via the remote.

## 🧠 System Architecture

The project is split into two distinct codebases running on two different architectures:

### 1. The Transmitter: Smart Remote (PIC32MX130)
* **Core:** MIPS32 architecture running at 40 MHz.
* **Inputs:** 2-Axis Analog Joystick and 4x digital push-buttons with hardware debouncing.
* **Outputs:** 16x2 LCD, audio speaker, status LEDs, and a 76kHz IR LED driven by `Timer2` interrupts.
* **Role:** Processes user inputs, calculates angular vectors and magnitudes, manages the UI state machine, and packages commands into discrete IR pulses.

### 2. The Receiver: Autonomous Rover (STM32L051xx)
* **Core:** ARM Cortex-M0+ running at 32 MHz.
* **Inputs:** IR Receiver diode (parsed via `TIM21`), 3x Magnetic Coils (ADC), VL53L0X ToF Sensor (I2C), and an IR proximity sensor.
* **Outputs:** 4x DC Motors (Tank drive configuration), 1x Servo Motor, and Bluetooth TX/RX, driven by custom software-defined PWM utilizing `TIM2` interrupts.
* **Role:** Decodes incoming IR packets, calculates motor PWM duty cycles, executes PID-style line following, manages Bluetooth telemetry, and halts operations based on sensor interrupts.

## 🛠️ Hardware Requirements

**Remote Controller:**
* Microcontroller: PIC32MX130F064B
* 2-Axis Analog Joystick
* 16x2 Character LCD
* IR Transmitter LED & Status LEDs
* Push buttons & Piezo Speaker

**Robot Chassis:**
* Microcontroller: STM32L051xx 
* Bluetooth Module (e.g., HC-05 / AT-09)
* 4x DC Motors & Motor Drivers (e.g., L298N)
* 1x Micro Servo (Claw)
* 3x Magnetic Induction Coils (Left, Right, Center/Intersection)
* VL53L0X Time-of-Flight Sensor
* IR Receiver Diode & Proximity Sensor

## 💻 Tech Stack

* **Languages:** Bare-Metal C
* **Protocols:** I2C (Sensors), Custom IR (Communication), UART (Bluetooth & Debugging)
* **Embedded Concepts:** Hardware Timers, Interrupt Service Routines (ISRs), ADC Polling, Software PWM generation, State Machines, Sensor Fusion.

## 🎮 Operating Modes

1. **Autonomous Mode:** The user selects a pre-programmed path or uploads a CSV via Bluetooth. The robot follows the magnetic tape, automatically executes turns, adjusts speed based on the Options Menu, picks up payloads, and auto-reverses if commanded. 
2. **Manual Mode:** The user swaps modes via joystick press. The PIC32 converts Cartesian coordinates into polar vectors and transmits them via IR, allowing fluid, proportional tank-style remote driving with manual claw control.
