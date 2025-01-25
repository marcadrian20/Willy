# WILLY: The Hexapod Robot

WILLY is a six-legged (hexapod) robot powered by an ESP32 microcontroller. Designed for versatility and control, WILLY can be operated via a web dashboard or an Xbox controller. Each leg has 3 Degrees of Freedom (DOF), allowing for precise and complex movements. He has an integrated balance controller based on pitch an roll. 

## This project is still WIP and given as is. 


## Features

- **ESP32 Powered:**
  - One ESP32 handles movement control.
  - Communication between two ESP32s uses ESP-NOW protocol for reliable, low-latency transmission.

- **Control Methods:**
  - **Web Dashboard:**
    - An intuitive interface to control WILLY remotely.
    - View and adjust movement parameters in real-time.
    - ![dashboard](https://github.com/user-attachments/assets/e280cccf-7d80-4438-9524-db62e25a32c1)
      
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

3. **3DOF:**
    - Each leg has 3 servo motors for movement.
    - Allows WILLY to perform walking, turning, and stability maneuvers effectively.

## Requirements

### Hardware
-
- 2x ESP32 microcontrollers(one is optional)
- 18x Servo motors (180-degree rotation, I have tested with cheap MG996R servos, any 180/270 servo should work)
- MPU6500 for telemetry and body temperature reading
- 2x PCA9685 for servo control(For now the robot is configured in quadruped mode so you can use 1 PCA9685 and 12 servos)
- Power supply and buck converter/voltage regulator for servos and ESP32
- Xbox controller(optional)

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
    - Modify in both firmwares the wifi SSID and password according to your needs
    - Modify the PID controller Kp,Ki,Kd according to your needs
    - Upload the BotController code to the first ESP32 (ESP for robot management)
    - Upload the XboxSender code to the second ESP32 (optional, for using with the XBOX controller)
    - Access the dashboard interface through the IP communicated on serial by the robot management ESP32.

4. **Testing:**
    - Test individual leg movements.
    - Test proper PID functionality
    - Verify communication between the Xbox controller and the ESP32 units.
    - Verify the dashboard functionality. Pitch and roll values especially 

## Getting Started

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/willy-hexapod.git
    ```
2. Install the required libraries in Arduino IDE or PlatformIO.
3. Upload the code to the ESP32 microcontrollers.
4. Access the web dashboard or pair your Xbox controller(modify the COM port in the RunXboxServer.py script and run).
5. Power the robot and start exploring!

## Future Improvements

- Implement obstacle avoidance using ultrasonic sensors or possibly LIDAR.
- Add more pre-programmed gaits for smoother movements. (For now only the wave gait works nicely)
- Add auto tuning for PIDs
- Expand the web dashboard with more telemetry data and functionality.
- Refactor codebase
- Add missing scripts

---
Feel free to reach out if you have any questions or ideas to improve WILLY!
