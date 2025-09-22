# ROS2 Node for Offboard Control
The `ros2_node/src` folder contains the ROS2 workspace for the autonomous offboard control of the T18 octorotor drone. The primary node, offboard_ctrl_example.py, handles mission-critical functions, including takeoff, smooth trajectory following, and landing. This ROS2 workspace was build from the following template: [PX4-ROS2-Gazebo-Drone-Simulation-Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template)

## File Structure
```
ros2_node
├── src/
│   ├── my_offboard_ctrl/                  # Custom Python ROS2 package
│   │   ├── my_offboard_ctrl/
│   │   │   ├── __init__.py
│   │   │   ├── gzcam.py                   # Gazebo camera handling node
│   │   │   └── offboard_ctrl_example.py   # Offboard control node
│   │   ├── resource/
│   │   │   └── my_offboard_ctrl
│   │   ├── package.xml                    # ROS2 package metadata
│   │   ├── setup.py                       # Python package setup
│   │   └── setup.cfg                      # Package configuration
│   └── px4_msgs/                          # PX4 ROS2 message definitions
└── README.md                              # This file
```

## Key Components
### `offboard_ctrl_example.py` – Offboard Control with Waypoint Following  

This ROS2 node demonstrates **offboard control of a PX4-based drone** in simulation.  
It publishes **high-level position setpoints** (`TrajectorySetpoint`) and control mode messages (`OffboardControlMode`) to PX4, which then uses its **internal flight controllers (PID-based position, velocity, and attitude loops)** to stabilize and track those setpoints.  

Control breakdown:
- **External Node (this script)**  
  - Generates a smooth trajectory from key waypoints.  
  - Continuously selects the next target point using a **lookahead-based trajectory tracking method**.  
  - Publishes desired position `(x, y, z)` and yaw setpoints at ~10 Hz.  
  - Manages mission phases (takeoff, waypoint following, landing).  

- **PX4 Autopilot (internal)**  
  - Runs **low-level PID control loops** for attitude, angular rate, velocity, and position.  
  - Receives setpoints from this ROS2 node and executes motor-level control to follow them.  
  - Handles flight stabilization, actuator mixing, and safety checks.  

Additional features in the script:
- **Takeoff logic** – Arms and climbs to a fixed altitude before starting trajectory following.  
- **Trajectory tracking** – Smoothly advances along the generated path rather than “jumping” between waypoints.  
- **Status monitoring** – Subscribes to `VehicleStatus` and `VehicleLocalPosition` to log flight state and provide progress updates.  
- **Visualization** – Records position history and produces a 3D plot of the flown path after mission completion.  

In short:
- **ROS2 node = path generator + high-level trajectory tracker.**  
- **PX4 = low-level controller (PID loops) that executes the commands.**  

### `gzcam.py` – Gazebo Camera Interface  

This Python module provides a simple interface for subscribing to a **Gazebo camera topic** and retrieving images for downstream processing. It uses the **Gazebo Transport API** (`gz.transport13`) to subscribe to camera messages and converts them into **OpenCV-compatible images**.  

Functionality:
- **Subscription to camera topic** – Connects to a Gazebo camera stream (`/camera` by default) using `gz.transport13.Node`.  
- **Image decoding** – Converts raw Gazebo image messages (`gz.msgs10.Image`) into `numpy` arrays, then into **OpenCV BGR images** for computer vision tasks.  
- **Thread-safe buffering** – Uses a `threading.Condition` to safely store and provide the latest camera frame without race conditions.  
- **Frame retrieval API** – `get_next_image()` blocks until a new image is available (with optional timeout), returning the latest frame.  
- **Visualization (example)** – In standalone mode (`__main__`), continuously displays the camera feed using OpenCV.  

This script acts as a **bridge between Gazebo and OpenCV**, making simulated camera images easily accessible for perception, computer vision, or autonomy pipelines.


### `package.xml` – ROS 2 Package Metadata  

This file defines the **metadata and build information** for the `my_offboard_ctrl` ROS 2 package. It follows the **ROS 2 package format 3** specification.  

This file tells ROS 2 how to **identify, build, and distribute** the `my_offboard_ctrl` package, and provides essential metadata for maintainability and compliance.

### `setup.cfg` – Installation Configuration  

This file specifies where Python scripts for the `my_offboard_ctrl` package should be placed during development and installation.  

Essentially, it ensures that ROS2 executables from this package are installed correctly and can be run using `ros2 run my_offboard_ctrl <executable>`.

### `setup.py` – Python Package Setup for ROS2

This file configures the `my_offboard_ctrl` Python package for ROS2:

- Defines **package metadata**: name, version, maintainer, license, and description.
- Uses `find_packages` to include all Python modules (excluding tests).
- Specifies **data files** required by ROS2, including `package.xml` and resource index entries.
- Declares **dependencies** (e.g., `setuptools`) and test requirements (`pytest`).
- Sets up **console scripts**, allowing ROS2 commands like:
  ```bash
  ros2 run my_offboard_ctrl offboard_ctrl_example
  ```
  to launch the offboard control node directly.
Essentially, `setup.py` makes this ROS2 Python package installable and executable within the ROS2 environment.