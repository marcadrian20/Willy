# WILLY: The Hexapod Robot

WILLY is a six-legged (hexapod) robot powered by an ESP32 microcontroller. Designed for versatility and control, WILLY can be operated via a web dashboard or an Xbox controller. Each leg has 3 Degrees of Freedom (DOF), allowing for precise and complex movements. 

## Features

- **ESP32 Powered:**
  - One ESP32 handles movement control.
  - Communication between two ESP32s uses ESP-NOW protocol for reliable, low-latency transmission.

- **Control Methods:**
  - **Web Dashboard:**
    - An intuitive interface to control WILLY remotely.
    - View and adjust movement parameters in real-time.
    - ![Web Dashboard Screenshot](screenshot.png)  
  - **Xbox Controller:**
    - Sends movement data to the first ESP32 via serial.
    - ESP32 relays commands to the second ESP32 controlling the robot.

- **Leg Configuration:**
  - 6 legs, each with 3DOF for advanced locomotion.
  - Adaptable to uneven terrain and complex movement patterns.

## System Architecture

1. **ESP-NOW Communication:**
    - Commands are transmitted from the controller ESP32 to the robot ESP32.
    - Fast and reliable communication.

2. **Control Modes:**
    - **Manual Control:** Using the Xbox controller for real-time interaction.
    - **Dashboard Control:** Accessible via any device with a browser.

3. **3DOF Mechanism:**
    - Each leg has 3 servo motors for movement.
    - Allows WILLY to perform walking, turning, and stability maneuvers effectively.

## Requirements

### Hardware
- 2x ESP32 microcontrollers
- 18x Servo motors (180-degree rotation)
- Power supply for servos and ESP32
- Xbox controller

### Software
- Arduino IDE or PlatformIO for programming the ESP32
- WebSocket library for dashboard communication
- ESP-NOW library for inter-ESP communication

## How to Build

1. **Hardware Assembly:**
    - Assemble the hexapod frame.
    - Mount the servo motors on each leg for 3DOF.
    - Connect the servos to the ESP32.

2. **Programming:**
    - Upload the ESP-NOW communication code to both ESP32 units.
    - Set up the web dashboard interface.

3. **Testing:**
    - Test individual leg movements.
    - Verify communication between the Xbox controller and the ESP32 units.

## Getting Started

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/willy-hexapod.git
    ```
2. Install the required libraries in Arduino IDE or PlatformIO.
3. Upload the code to the ESP32 microcontrollers.
4. Access the web dashboard or pair your Xbox controller.
5. Power the robot and start exploring!

## Future Improvements

- Implement obstacle avoidance using ultrasonic sensors.
- Add pre-programmed gaits for smoother movements.
- Expand the web dashboard with telemetry data.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments
- Open-source communities for libraries and tutorials.
- Inspiration from other hexapod projects.

---
Feel free to reach out if you have any questions or ideas to improve WILLY!
