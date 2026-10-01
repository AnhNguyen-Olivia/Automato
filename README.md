# Automato 

> A robotics simulation project focused on developing an autonomous tomato harvesting system using **Unity, ROS, a UR10e robotic arm, and an OnRobot RG2 gripper**.

![Status](https://img.shields.io/badge/Status-In_Development-yellow)
![License](https://img.shields.io/badge/License-MIT-green)
![OS](https://img.shields.io/badge/OS-Ubuntu_20.04_LTS-orange)

---

## Table of Contents
- [Project Overview](#project-overview)
- [Team Members](#team-members)
- [Project Objectives](#project-objectives)
- [Technology Stack](#technology-stack)
- [System Architecture](#system-architecture)
- [Development Roadmap](#development-roadmap)
- [Repository Structure](#repository-structure)
- [Getting Started & Installation](#getting-started--installation)
- [How to Run the Simulation](#how-to-run-the-simulation)
- [Testing and Validation](#testing-and-validation)
- [Project Status](#project-status)
- [Third-Party Assets and Acknowledgments](#third-party-assets-and-acknowledgments)
- [License](#license)

---

## Project Overview

The goal of this project is to simulate and develop an autonomous robotic system capable of identifying tomatoes, planning a harvesting motion, and picking tomatoes using a robotic arm and gripper.

The project builds on an existing Unity pick-and-place simulation baseline and extends it into a full agricultural harvesting workflow, including:
* Tomato detection & ripeness classification
* 3D stem/peduncle localization
* Precision grasping 
* Static and dynamic obstacle avoidance

Unity serves as the 3D physics simulation environment, while ROS handles high-level motion planning, coordinate transformations, and control software.

---

## Team Members

| Name | Student ID | GitHub Profile |
| :--- | :--- | :--- |
| **Nguyen Thuy Anh** | `10423198` | [@AnhNguyen-Olivia](https://github.com/AnhNguyen-Olivia) |
| **Tran Anh Thu** | `10423173` | [@anhthu-tat](https://github.com/anhthu-tat) |
| **Truong Nguyen Nha Han** | `10423038` | [@Jo-T2583](https://github.com/Jo-T2583) |
| **Nguyen Bao Thanh Dat** | `10423136` | [@ThDat4505](https://github.com/ThDat4505) |

---

## Project Objectives

* Integrate a **UR10e robotic arm** into the Unity simulation environment.
* Integrate and control an **OnRobot RG2 gripper** end-effector.
* Establish communication between Unity and ROS via TCP bridge.
* Develop tomato detection and estimation/localization components.
* Implement a pick-and-place harvesting workflow.
* Develop static and dynamic obstacle-avoidance capabilities.
* Validate system performance through simulation milestones and documented testing.

---

## Technology Stack

|## Technology Stack

| Technology | Purpose | Notes / Version |
| :--- | :--- | :--- |
| **Unity** | 3D simulation environment | `[ADD: Unity Version]` |
| **ROS 1 Noetic** | Robotics middleware | Ubuntu 20.04 LTS execution layer |
| **MoveIt** | Motion planning and robot manipulation | Trajectory generation & collision checking |
| **UR10e** | Robotic arm | 6-DOF collaborative arm model |
| **OnRobot RG2** | Gripper | Parallel mechanical gripper model |
| **ROS-TCP-Connector** | Unity–ROS communication | High-speed TCP communication bridge asset |
| **WSL2 / Docker** | ROS development environment | Linux container/subsystem environment on Windows |
| **Perception** | Vision / Detection stack | `[ADD: Vision library, e.g., YOLOv8 + OpenCV]` |

---

## Development Roadmap

The project has three main milestones.

### M1 — Tomato Harvesting

* [ ] Verify the existing Unity pick-and-place baseline.
* [ ] Integrate the UR10e robotic arm.
* [ ] Integrate the OnRobot RG2 gripper.
* [ ] Establish Unity–ROS communication.
* [ ] Implement tomato pick-and-place behavior.
* [ ] Test and document the harvesting workflow.

### M2 — Static Obstacle Avoidance

* [ ] Add static obstacles to the simulation.
* [ ] Configure the robot planning environment.
* [ ] Implement and test collision-aware motion planning.
* [ ] Document the test results.

### M3 — Dynamic Obstacle Avoidance

* [ ] Introduce moving obstacles into the simulation.
* [ ] Develop obstacle-aware motion behavior.
* [ ] Test the robot's response to changing obstacles.
* [ ] Record and document the results.

> Milestone checklists will be updated as features are implemented and checked.

## Repository Structure

```text
TBD
```

*Note: The structure above is the planned organization. Directories and ROS packages will be added as development progresses.*

## Getting Started

### Prerequisites

Before installing the project, ensure you have the following configured:
Development Environment: WSL2 or Docker running Ubuntu 20.04 LTS.
ROS Distribution: ROS 1 Noetic Desktop Full (Installation Guide).
Unity Engine: [PLEASE INSERT: Unity Version, e.g., Unity 2020.3 LTS / 2022.3 LTS] installed via Unity Hub.
Dependencies: Catkin build tools, MoveIt packages, and Git.

### Setup/Installation Steps

Setup instructions are under development.

Please refer to the project documentation for environment configuration, Unity setup, ROS workspace setup, and instructions for running the simulation.

* [Setup Guide] TBD
* [System Architecture] TBD
* [Testing Guide] TBD

## How to run the simulation

## Testing and Validation

The project is developed and validated incrementally through simulation.

Testing evidence may include:

* Unity simulation screenshots
* RViz visualization
* ROS node and topic communication
* Pick-and-place test results
* Obstacle-avoidance test results
* Demo videos and logs

Features will be marked as completed only after they have been tested.

## Project Status

### Status: In development

The project begins with an existing Unity pick-and-place baseline. UR10e + RG2 integration and subsequent harvesting and obstacle-avoidance capabilities are part of the development roadmap.

## Third-Party Assets and Acknowledgments

This project may use third-party robotics packages, models, and assets.

Their original authors, licenses, and attribution requirements will be documented in `THIRD_PARTY_NOTICES.md`.

Tomato model attribution and license information will be included according to the original asset's license.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

Third-party packages and assets remain subject to their respective licenses.



