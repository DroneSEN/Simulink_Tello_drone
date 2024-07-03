# Tello Edu Control Software on Simulink

## Table of Contents

1. [Setup the Project](#setup-the-project)
2. [Materials Used](#materials-used)
3. [Tello Drone Control on Simulink](#tello-drone-control-on-simulink)
4. [Modified Tello Matlab Library](#modified-tello-matlab-library)
5. [SLAM (Simultaneous Localization and Mapping) on Simulink with Tello](#slam-on-simulink-with-tello)
6. [Optitrack Setup](#optitrack-setup)
7. [Drone Positioning with YOLO](#drone-positioning-with-yolo)
8. [Semantic Mapping with YOLO](#semantic-mapping-with-yolo)
9. [Camera Calibration with Matlab](#camera-calibration-with-matlab)
10. [Aruco Positioning with Tello's Down Camera](#aruco-positioning-with-tellos-down-camera)
11. [Waypoint Manager and Control of Tello](#waypoint-manager-and-control-of-tello)
12. [Bibliography](#bibliography)

## Setup the Project

1. Clone the Github repository : git clone [repository_url]
2. Go inside the directory
3. Install the dependencies with the command `git submodule update --init`
   This will install the Optilink module, used to link Optitrack to the Simulink model.
4. Setup the Optilink tool (Cf. Readme/Setup on https://github.com/DroneSEN/Optilink)

## Materials Used

- Tello EDU Drones: Used for implementing control algorithms and SLAM.
- Simulink: A simulation and model-based design environment for dynamic and embedded systems.
- Optitrack System: Used for precise tracking of the drone.
- YOLO (You Only Look Once): A real-time object detection system.
- Matlab: For camera calibration, SLAM, and various computational tasks.
- Aruco Markers: Used for positioning with the Tello's down-facing camera.

## Tello Drone Control on Simulink

## Modified Tello Matlab Library

## SLAM (Simultaneous Localization and Mapping) on Simulink with Tello

## Optitrack setup

## Drone Positioning with YOLO

## Semantic Mapping with YOLO

## Camera Calibration with Matlab

## Aruco Positioning with Tello's Down Camera

## Waypoint Manager and Control of Tello

## Bibliography

Carlos Campos et al., *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM* [Link](https://arxiv.org/pdf/2007.11898)

Peter Corke, *Robotics, Vision and Control (Third Edition), Fundamental Algorithms in Python* [Link](https://link.springer.com/book/10.1007/978-3-031-06469-2)

Peter Corke, *Robotics, Vision and Control (Second Edition), Fundamental Algorithms in Matlab* [Link](https://link.springer.com/book/10.1007/978-3-319-54413-7)


Steven M. LaValle, *Planning Algorithms* [Link](https://lavalle.pl/planning/)