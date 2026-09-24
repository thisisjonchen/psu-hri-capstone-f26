# PALS: Paired Aerial Lift System
> *A joint collaboration between Penn State University (PSU) and Honda Research Institute (HRI)*

PALS is a cooperative multi-drone system in which two Duckiedrone DD24 quadcopters autonomously transport a shared cable-suspended payload.

<image src="diagrams/hl-diagram.png" alt="Display"/>

We have 3 core objectives:
- **Autonomous Payload Transport**: Demonstrate coordinated autonomous takeoff, payload lift, point-to-point transport over a predefined route, controlled delivery, and drone landing without human intervention.
- **Inter-Drone Coordination**: Implement communication protocols between drones to enable the exchange of sensor data and mission states. Drones should remain in a stable flight configuration and limit payload motion during transport.
- **Safety and Fault Handling**: Detect predefined abnormal conditions, such as communication loss, unsafe position, or obstructions. Execute a safe response such as holding position, lowering payload, or terminating the mission. 


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

## Stacks

### Hardware
<image src="diagrams/hardware-diagram.png" alt="Display"/>

- 2× Duckiedrone DD24-B
- Raspberry Pi 4 onboard computers
- onboard cameras and Time-of-Flight sensors
- optical-flow sensors
- shared cable and payload mechanism

### Software
- **Python** — autonomy, perception, and coordination
- **ROS 2** — onboard and inter-drone communication
- **MAVROS / MAVLink** — flight-controller interface
- **PX4** — low-level flight control
- **OpenCV / AprilTag** — visual localization

## Acknowledgments
### PJ1E Team
- **Arjun Gupta**, Computer Engineer​
- **Aiden Derr**, Computer Engineer​
- **Jacob Meert**, Computer Engineer
- **V V Shivkumar**, Computer Scientist
- **Louis Nguyen**, Computer Engineer
- **Jon Chen**, Software Engineer
  
### Honda Research Institute (Sponsor)
- **Brain Coy**, Principal Research Engineer at HRI
- **Rajeev Chhajer**, Chief Engineer / Group Lead at HRI
- **Ryan Lingo**, Applied AI Engineer / Dev Advocate at HRI

### Penn State College of Engineering
- **Kyusun Choi**, Professor and Advisor

