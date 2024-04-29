# Dream Mobile Platform

I have written [a blog post](https://ricoruotongjia.medium.com/set-up-raspberry-pi-4b-as-a-mobile-platform-7448e94a04dc) in terms of setting this up

## Core Features and Software Rationale

This package is a "one-stop-shop" for Rpi setup. That is, all functionalities are modular, and easy to start and end.

All our robot services are run inside [Byobu](https://www.byobu.org/) in a docker container. Our simulation nodes are run **outside** the docker container, and can be run on a Ubuntu 20.04 machine.

Features

- LED Toggling
- Low Level Motor Controller
- Motor PID control
- Keyboard gamepad direction controller

## Set up

1. Go through the set up steps [../../README.md] for basic Rpi set up
2. See the comments section in [Dockerfile](./Dockerfile) to pull the image and run a container
   - The container should have everything you need to running all functionalities
3. Check feature flags defined in [the feature flag file](./dream_feature_flags).
4. Source the feature flag file `source dream_feature_flags`
5. If you want to start simulation:
   ```bash
   roslaunch dream_mobile_platform low_level_drivers.launch
   ```
   - Or on the robot, start the byobu services using "dream_byobu". `dream_byobu <-l> <-b>`
     - Each input arg is optional. See documentation with `dream_byobu -h`
6. Onboard package building
    - We are skipping catkin-building some offline packages. Check out their CMakeLists.txt

### Simulation

- The keypad node is launched on the same console. However, one should note that the node will capture motions on other consoles as well.
- To record rosbags: `rosbag record /dream/scan /tf /dream/wheel_pos`

## Motor Control

Production Digram

```mermaid
graph TB
    subgraph "Onboard container"
        D
        B
        A
        C
    end

    subgraph "Local Simulation"
        GAZEBO
        GAZEBO_DIFF_DRIVE
        GAZEBO_LIDAR
    end

    subgraph "SLAM/localization"
        GMAPPING/RMCL
    end

D["MOTOR_ENCODER"] -->|/ROS_TOPIC/WHEEL_POS| GMAPPING/RMCL

A["MOTOR_CONTROLLER"] -->|/SHM_TOPIC/MOTOR_COMMANDS| B["MOTOR_DRIVER"] --> C["MOTORS"] --> D["MOTOR_ENCODER"] --> |/SHM_TOPIC/WHEEL_VELOCITIES|A

F["Keyboard Gamepad"] -->|/ROS_TOPIC/CMD_VEL|CONNECTOR_CMD_VEL_NODE{ } --> G["cmd_vel_to_motor_commands"] -->|/SHM_TOPIC/COMMANDED_WHEEL_VELOCITY| CONNECTOR_WHEEL_VEL_NODE{ }
        - We are skipping catkin-building some offline packages. Check out their CMakeLists.txt
CONNECTOR_WHEEL_VEL_NODE --> A
CONNECTOR_WHEEL_VEL_NODE--> GAZEBO_DIFF_DRIVE --> GAZEBO
GAZEBO_LIDAR --> GAZEBO

H["LOCAL_PLANNER"] -->CONNECTOR_CMD_VEL_NODE
```

Motor Tuning: please see [the motor tuning script](scripts/tune_motor_controller.py)

### Usage

- Modify `/PARAMS/DEBUG_MOTORS` in dream_resource_registry so we can print debug messages on the spot
