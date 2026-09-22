# Automato

A robotics simulation project focused on developing a tomato harvesting system using **Unity, ROS, a UR10e robotic arm, and an OnRobot RG2 gripper**.

The project builds on an existing Unity pick-and-place simulation and aims to extend it into a robotic harvesting workflow, including tomato detection, localization, grasping, and obstacle avoidance.

## Project Overview

The goal of this project is to simulate and develop a robotic system capable of identifying tomatoes, planning a harvesting motion, and picking tomatoes using a robotic arm and gripper.

Unity is used as the simulation environment, while ROS provides the communication and robotics software framework.

The project follows an incremental development approach, starting with a working pick-and-place baseline and progressing toward more advanced harvesting and obstacle-avoidance capabilities.

## Team Members

| Name                   | ID       |  GitHub                                                  |
|------------------------|----------|----------------------------------------------------------|
| Nguyen Thuy Anh        | 10423198 | [@AnhNguyen-Olivia](https://github.com/AnhNguyen-Olivia) |
| Tran Anh Thu           | 10423173 | [@anhthu-tat](https://github.com/anhthu-tat)             |
| Truong Nguyen Nha Han  | 10423038 | [@Jo-T2583](https://github.com/Jo-T2583)                 |
| Nguyen Bao Thanh Dat   | 10423136 | [@ThDat4505](https://github.com/ThDat4505)               |

## Project Objectives

* Integrate a UR10e robotic arm into the Unity simulation.
* Integrate and control an OnRobot RG2 gripper.
* Establish communication between Unity and ROS.
* Develop tomato detection and localization components.
* Implement tomato pick-and-place and harvesting behavior.
* Develop static and dynamic obstacle-avoidance capabilities.
* Validate the system through simulation and documented testing.

## Technology Stack

| Technology        | Purpose                                |
| ----------------- | -------------------------------------- |
| Unity             | 3D simulation environment              |
| ROS 1 Noetic      | Robotics middleware                    |
| MoveIt            | Motion planning and robot manipulation |
| UR10e             | Robotic arm                            |
| OnRobot RG2       | Gripper                                |
| ROS-TCP-Connector | Unity–ROS communication                |
| WSL2 / Docker     | ROS development environment            |

## Development Roadmap

The project is organized into three main milestones.

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

> Milestone checklists will be updated as features are implemented and verified.

## Repository Structure

```text
TBD
```

*Note: The structure above is the planned organization. Directories and ROS packages will be added as development progresses.*

## Getting Started

### Prerequisites

* Unity (version to be specified)
* ROS 1 Noetic
* MoveIt
* WSL2 or Docker (for the ROS environment)
* Git

### Setup

Setup instructions are under development.

Please refer to the project documentation for environment configuration, Unity setup, ROS workspace setup, and instructions for running the simulation.

* [Setup Guide] TBD
* [System Architecture] TBD
* [Testing Guide] TBD

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
