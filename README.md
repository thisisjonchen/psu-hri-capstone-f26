# PALS — Paired Aerial Lift System

PALS is a cooperative multi-drone system in which two Duckiedrone DD24 quadcopters autonomously transport a shared cable-suspended payload.

## System Overview

```text
Payload Detection / Pickup
          ↓
Coordinated Positioning
          ↓
      Payload Lift
          ↓
Cooperative Transport
          ↓
     Payload Delivery
          ↓
     Return to Base
```

## Perception

The perception subsystem is responsible for:

- Localization
- Altitude and obstacle sensing using Time-of-Flight sensors
- Indoor motion estimation
- Payload and pickup/drop-off zone detection

## Planning & Control

The planning and control subsystem is responsible for:

- Inter-drone state communication
- Coordinated takeoff and landing
- Formation and trajectory control
- Payload stabilization
- Obstacle and fault handling
- Autonomous pickup and delivery

## Hardware

- 2× Duckiedrone DD24-B
- Raspberry Pi 4 onboard computers
- onboard cameras and Time-of-Flight sensors
- optical-flow sensors
- shared cable and payload mechanism

## Software Stack

- **Python** — autonomy, perception, and coordination
- **ROS 2** — onboard and inter-drone communication
- **MAVROS / MAVLink** — flight-controller interface
- **PX4** — low-level flight control
- **OpenCV / AprilTag** — visual localization
