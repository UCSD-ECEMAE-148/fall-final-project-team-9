# 148-fall-2025-final-project-fall-2025-final-project-team-9
148-fall-2025-final-project-fall-2025-final-project-team-9 created by GitHub Classroom
# Parallel Parker with Occupancy Identification and Object Remover 

### Team Members
* **[Name]** - [Major/Role] - [Class Year]
* **[Name]** - [Major/Role] - [Class Year]
* **[Name]** - [Major/Role] - [Class Year]
* **Owen Hanenian** - Mechanical Engineering - Class of 2026

---

## Overview
[Provide a high-level summary of the project. What is the goal? What problem are you solving? Briefly mention the core technology stack.]

## Abstract
Our goal was to have the car traveling on a straight path, with parking spots parallel to it's path (one on it's left, one on 
it's right). Based on the occupancy of these spots (using a "No Parking" sign), the car decides which spot is open and parks in that spot using a "Look, Measure, then Execute" type of strategy. 
The system was implemented within a ROS2 framework, utilizing an OAK-D Lite camera for depth perception and OpenCV for real-time computer vision processing.
Additionally, a sweeper mechanism was installed using a servo motor controlled by a PCA9685 driver to remove interfering objects.  

## Key Features
* **Feature 1:** [Brief description]
* **Feature 2:** [Brief description]
* **Feature 3:** [Brief description]

## Core Objectives
* **Objective A:** [e.g., Implement a tracking node...]
    * *Measure:* [How do you quantify success?]
    * *Action:* [What does the system do with this data?]
* **Objective B:** [e.g., Robot remains stationary until...]

## 4. System Architecture
### Node Descriptions
* **`[node_name].py`**
    * **Inputs:** [e.g., Camera feed, Lidar scan]
    * **Logic:** [Briefly explain the processing]
    * **Outputs:** [e.g., Motor control commands]
* **`[node_name].py`**
    * **Inputs:** [...]
    * **Outputs:** [...]

## 5. Hardware & Embedded Systems
* **Compute Unit:** [e.g., Jetson Nano] running [OS].
* **Sensors:** [List sensors used].
* **Connectivity:** [e.g., Wireless SSH].
* **CAD/Mechanical:**
    * *Custom Parts:* [List parts you designed]
    * *Stock Parts:* [List standard components]

## 6. Setup & Execution
1. **Environment:** `[command to source environment]`
2. **Build:** `colcon build --packages-select [package_name]`
3. **Launch:** `ros2 launch [package_name] [launch_file.py]`

## 7. Future Improvements
* [Improvement 1]
* [Improvement 2]

## 8. Acknowledgments
* [Credit professors, TAs, or open-source libraries]
