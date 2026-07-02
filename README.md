# Project: Development of a 6-DOF Anthropomorphic Robotic Arm with Motion Interpolation Algorithms

## Project Overview

The objective of this project is to design, build, and control a six-degree-of-freedom (6-DOF) anthropomorphic robotic arm capable of executing smooth and accurate motion. The system implements both joint-space interpolation and Cartesian linear interpolation, enabling coordinated manipulator movement in joint and task space.

The project encompasses the complete development process, including mechanical design, hardware integration, embedded control, robot kinematics, trajectory generation, and system validation. The robotic arm is designed as a modular, research-oriented platform suitable for experimentation with industrial robotics concepts and motion planning algorithms.

## Recommended Literature

* John J. Craig, *Introduction to Robotics: Mechanics and Control*
* Jerzy Honczarenko, *Industrial Robots*
* Karol Kozłowski, *Modeling and Control of Robots*

## Technology Stack

### 1. Hardware

* **Host Computer:** Raspberry Pi 5 or PC
* **Actuators:** Waveshare ST3020 Serial Bus Servo Motors
* **Servo Controller:** ESP32 Servo Driver Expansion Board
* **Power Supply:** 12 V / 25 A DC Power Supply
* **Mechanical Structure:** 3D-printed PETG components, radial ball bearings, and a thrust bearing for the base joint

### 2. Software

* **CAD Software:** Autodesk Fusion
* **Operating System:** Windows/Linux
* **Robotics Framework:** ROS 2 (Humble Distribution)

  * **Motion Planning:** MoveIt 2
  * **Visualization:** RViz
* **Programming Languages:**
  * Python (simulation)
  * C++ / Python (ROS 2)
  * C/C++ (ESP32 firmware)

### 3. Communication

* **Servo Controller ↔ Servo Motors:** UART (Single-Wire Half-Duplex) for position commands and servo feedback
