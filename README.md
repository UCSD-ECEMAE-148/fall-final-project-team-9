#MAE148 Fall 2025 - Team 9 - Smart Parallel Parking Robot
MAE148 Fall 2025 - Team 9 - Smart Parallel Parking Robot created by GitHub Classroom
# Parallel Parking Car with Sign Identification and Obstacle Remover

### Team Members
* **Trew Hoffman** - Mechanical Engineering: Software - Class of 2026
* **Manan Tuteja** - Aerospace Engineering: Hardware - Class of 2026
* **Terri Tai** - Computer Engineering: Software - Class of 2025
* **Owen Hanenian** - Mechanical Engineering: Hardware - Class of 2026

---

## Overview
This project implements an autonomous parallel-parking robot that identifies which of two parallel parking spots is available and executes a full parking maneuver accordingly. The robot uses a vision-based decision pipeline, detecting no-parking signs, measuring depth to an AprilTag, and chooses the correct side to park.

The full system runs on ROS2, uses an OAK-D Lite for stereo depth + RGB vision, controls the car through a custom Hardware Abstraction Layer (HAL) connected to a VESC, and includes a servo-driven sweeper mechanism for obstacle removal.

The design intentionally follows a “Look → Measure → Execute” architecture, ensuring that every parking action is informed by sensor data.

## Abstract
Our goal was to create an autonomous ground robot that travels along a straight hallway with two potential parking spaces—one on the left and one on the right. A red NO PARKING sign declares one spot as occupied, and the robot must determine which side is open, measure its distance to the AprilTag marking the correct parking spot, and perform a tight parallel-parking sequence.

The system relies on the OAK-D Lite’s depth and rectified stereo output to estimate the true distance to an AprilTag using a median-over-area approach. The RGB stream is used to detect large red circular regions (NO PARKING signs) or, in ambiguous cases, a left-versus-right red-pixel imbalance.

Once the spot is selected, the orchestrator executes a multi-step parking routine using the HAL, steering, reversing, and straightening until the robot is aligned within its chosen spot. A sweeper arm powered by a servo motor controlled via a PCA9685 removes debris blocking the parking path.

## Key Features
* **Vision-based sign detection:** Identifies red circular no-parking signs or left/right red distribution imbalance to determine which spot is closed via HSV mask. 
* **AprilTag depth measurement:** [Computes tag distance using OAK-D rectified stereo depth, median-filtered for accuracy.
* **Parallel parking maneuver generator:** Performs a multi-stage, pre-tuned parking sequence via HAL steering and motor control.
* **ROS2 Modular architecture:** Clean separation of HAL, vision, orchestrator, and servo control nodes.

## Core Objectives
* **Objective A:** Identify the correct parking spot
    * *Measure:* Detect presence and location of no-parking sign
    * *Action:* Return via ROS2 service which side is available, with no sign present
* **Objective B:** Execute a parallel parking maneuver to the chosen spot
    * *Measure:* Receive desired side from vision node
    * *Action:* Execute hard-coded parallel parking manever with distances and steering dependent on chosen parking spot side.
* **Objective C:** Determine distance to parking spots
    * *Measure:* Use OAK-D stereo depth to determine distance to AprilTag (parking spots)
    * *Action:* Drive forward the distance determined by OAK-D measurement to appraoch parking spots. Use an iterative approach to get closer if the AprilTag is too far to determine distance. 

## 4. System Architecture
### Node Descriptions
* **`vision_node.py`**
    * **Inputs:**
        * RGB preview, mono left/right, stereo depth from OAK-D
        * Trigger service call /get_parking_spots
    * **Logic:**
        * Detect red circular NO PARKING signs or red imbalance.
        * Detect AprilTag (tag36h11) in rectified-left frame.
        * Compute median depth inside tag polygon.
        * Apply linear depth correction.
        * Return available side and corrected tag depth.
    * **Outputs:**
        * Trigger response: "side={left/right} depth_m={float}"
* **`orchestrator.py`**
    * **Inputs:**
        * Output from parking_vision_node
        * ROS 2 timer or driver logic to initiate maneuver
    * **Logic:**
        * Call vision service.
        * Parse "side" and "depth_m".
        * Execute parallel_park_left() or parallel_park_right() with correct HAL commands.
    * **Outputs:**
        * Motion commands to HAL for steering and driving.
* **`HAL.py`**
    * **Inputs:**
        * Calls to drive() and set_steering() commands of certain distances and normalized steering angle.
    * **Logic:**
        * Convert high-level movement commands into VESC duty cycle and servo commands.
    * **Outputs:**
        * Motor power and steering PWM to the car. 

## 5. Hardware & Embedded Systems
* **Compute Unit:** Raspberry Pi 5 running Rasperry Pi OS (Bookworm)
* **Sensors:**
  * OAK-D Lite
  * VESC Hall sensors
  * AprilTag fiducial marker
* **Connectivity:**
  * Ethernet SSH, wireless SSH
* **CAD/Mechanical:**
    * *Custom Parts:*
      * Sweeper arm
      * Mount for OAK-D Lite
    * *Stock Parts:*
      * DC-DC converter
      * VESC
      * 2 Servos (steering + arm)
      * Car chassis

## 6. Setup & Execution
1. **Environment:** `source ~/ros2_ws/install/setup.bash`
2. **Build:** `colcon build --packages-select parking_bot`
3. **Launch:** `ros2 launch parking_bot parking.launch.py`

## 7. Future Improvements
* Fix implementation of sweeper arm and obstacle detection in vision node that was not completed.
* Utillize LiDAR for parking spot, sign, and obstacle detection rather than stereo depth and RGB with color masks. 

## 8. Acknowledgments
* Dr. Jack Silberman, Winston Chou
