# Octorotor Drone Simulation Environment for Forest Monitoring (ROS2, PX4, Gazebo)

![Overview gif](media/1.gif)

This repository contains the simulation environment for the Tarot T18 octorotor drone designed for plant health monitoring using PX4, Gazebo, and ROS2.

## Table of Contents

- [Simulation Setup](#simulation-setup)
- [Prerequisites Installation](#prerequisites-installation)
- [Important Notes](#important-notes)
- [Installation](#installation)
- [Verification](#verification)
- [Running the Simulation](#running-the-simulation)

## Simulation Setup
- Ubuntu 22.04 (Jammy Jellyfish)
- PX4 v1.15.4
- Gazebo Harmonic
- ROS2 Humble
- Micro XRCE-DDS Agent
- QGroundControl

## Prerequisites Installation

For detailed installation instructions, please refer to:
- [PX4 / ROS2 Humble Installation](https://docs.px4.io/main/en/ros2/user_guide.html#install-px4)
- [Gazebo Harmonic Installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [Micro XRCE-DDS Agent Installation](https://docs.px4.io/main/en/middleware/uxrce_dds.html#build-run-within-ros-2-workspace)
- [QGroundControl Installation](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

### Important Notes:

**PX4 Version Compatibility**: The newest version of PX4 may have compatibility issues with this simulation environment. PX4 v1.15.4 is recommended as it has been tested and verified to work. If you have already installed a different version of PX4, you can change to v1.15.4 using:

```bash
cd ~/PX4-Autopilot
git checkout v1.15.4
git submodule update --init --recursive
make px4_sitl
```
Verify the version using:
```bash
cd ~/PX4-Autopilot
git describe --tags
```

**Micro XRCE-DDS Agent Installation**: When installing the Micro XRCE-DDS Agent, use the **"Build/Run within ROS 2 Workspace"** installation procedure as described in the [PX4 documentation](https://docs.px4.io/main/en/middleware/uxrce_dds.html#build-run-within-ros-2-workspace). This simulation environment was built and tested using this installation method.

## Installation

### Step 1: Clone the Repository

First, clone this repository to your home directory:

```bash
cd ~
git clone fsc_octorotor_simulation_ros2_px4_gazebo
```

This step downloads all the necessary simulation files including:
- px4_msgs that matches the PX4-Autopilot version
- Custom octorotor airframe configuration file
- Gazebo models and world
- ROS2 node for autonomous waypoint following

### Step 2: Run the Installation Script

Navigate to the repository and run the installation script:

```bash
cd ~/fsc_octorotor_simulation_ros2_px4_gazebo
chmod +x install_t18_simulation.sh
./install_t18_simulation.sh
```

The installation script performs four main setup tasks:

**px4_msgs Installation**
- Ensures ROS2 uses correct PX4 message definition by:
  - First checking if the `px4_msgs` repository already exists in the `~/fsc_octorotor_simulation_ros2_px4_gazebo/ros2_node/src` directory.
  - If the repository is not found, the script clones it from the official PX4 GitHub ([px4_msgs](https://github.com/PX4/px4_msgs/tree/95a7c089782afe8d92fee8cd68c50b9ef2cfbd65)).
  - If it exists, the script updates it to match the exact version (v1.15.4) required for this simulation.
- This allows proper communication between ROS2 and PX4
- See the [ROS2 README](ros2_node/src/README.md) for details

**PX4 Airframe Setup:**
- Copies the custom `4012_gz_t18` airframe configuration file to `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`
- Registers the new airframe in PX4's CMakeLists.txt so it appears in available airframes
- This allows PX4 to recognize and use the T18 octorotor configuration
- See the [airframe configuration README](px4_airframe_configuration/README.md) for details.

**Gazebo Model Installation:**
- Installs folders containing 3D models of the octorotor drone to `~/PX4-Autopilot/Tools/simulation/gz/models/`:
  - `t18` - Main T18 octorotor model
  - `t18_base` - Base variant of T18
  - `t18_mono_cam` - T18 with monocular camera
  - `Pine Tree` - Tree model for creating forest world ([Original Pine Tree Model](https://app.gazebosim.org/OpenRobotics/fuel/models/Pine%20Tree))
- See the [Gazebo Model README](gazebo/gazebo_model/README.md) for details

**Gazebo World Setup:**
- Installs the `customforest.sdf` world file to `~/PX4-Autopilot/Tools/simulation/gz/worlds/`
- Registers the world in PX4's simulation CMakeLists.txt at `~/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt`
- This creates a realistic forest testing environment for the T18 drone
- See the [Gazebo World README](gazebo/gazebo_world/README.md) for details

### Verification

After running the installation script, you should see messages confirming:
- px4_msgs reset/clone
- Airframe installation/update
- Airframe registration in CMakeLists.txt
- Gazebo model installations/updates (t18, t18_base, t18_mono_cam, Pine Tree)
- Gazebo world installation/update
- World registration in CMakeLists.txt

## Running the Simulation
### Simulating Onboard Control with QGroundControl
#### 1. Launch PX4 SITL and Gazebo:
```bash
cd ~/PX4-Autopilot/ 
PX4_SYS_AUTOSTART=4012 \
PX4_SIM_MODEL=gz_t18 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_GZ_WORLD=customforest \
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```
Note that 4012 refers to the custom airframe configuration. Either ```gz_t18``` (T18 octorotor) or ```gz_t18_mono_cam``` (T18 with monocular camera) can be selected depending on the simulation purpose.
Note that by right clicking on the drone in Gazebo, you can set it to automatically follow the drone.
#### 2. Open QGroundControl and set the desired path
See the following tutorial for details: [QGroundControl Tutorial](https://www.youtube.com/watch?v=0d23O_RUOmI&t=866s)
#### 3. Upload mission from QGroundControl
#### 4. Slide to Confirm mission execution

### Simulating Offboard Control with ROS2 Node
#### 1. Initialize Micro XRCE Agent:
```bash
cd ~/px4_ros_uxrce_dds_ws/
source /opt/ros/humble/setup.bash
source install/local_setup.bash 
MicroXRCEAgent udp4 -p 8888
```
#### 2. Launch PX4 SITL and Gazebo:
```bash
cd ~/PX4-Autopilot/ 
PX4_SYS_AUTOSTART=4012 \
PX4_SIM_MODEL=gz_t18 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_GZ_WORLD=customforest \
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```
Note that either `gz_t18` (T18 octorotor) or `gz_t18_mono_cam` (T18 with monocular camera) can be selected depending on the simulation purpose.  
Wait until you see the message **`INFO  [uxrce_dds_client] time sync converged`** in the PX4 terminal before running the ROS2 node.
#### 3. Execute ROS2 Node:
```bash
cd ~/fsc_octorotor_simulation_ros2_px4_gazebo/ros2_node
colcon build
source install/local_setup.bash
ros2 run my_offboard_ctrl offboard_ctrl_example
```
Note: The first build may take some time to complete (up to ~10 minutes).

## Troubleshooting
- Make sure that `PX4-Autopilot`, `fsc_octorotor_simulation_ros2_px4_gazebo`, and `px4_ros_uxrce_dds_ws` are all located in your home directory (`~/`).
- You may encounter the following error messages when launching Gazebo:
  ```bash
  ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.
  ERROR [gz_bridge] Task start failed (-1)
  ERROR [init] gz_bridge failed to start and spawn model
  ERROR [px4] Startup script returned with return value: 256
  ```
  In some cases, Gazebo may also fail to close properly when pressing `Ctrl+C`.
  If this happens, you can manually terminate all Gazebo processes by running:
  ```bash
  sudo pkill -9 -f gz
  ```
  After killing the processes, restart the simulation. It may take a few attempts before Gazebo launches successfully.
- The drone may hover and never reach the first waypoint. This is most likely due to a communication error between ROS2 and PX4. Ensure that the `PX4-Autopilot` version and `px4_msgs` version matches. To check the versions, run:
  ```bash
  cd ~/PX4-Autopilot
  git describe --tags --always
  git branch --show-current
  ```
  ```bash
  cd ~/fsc_octorotor_simulation_ros2_px4_gazebo/ros2_node/src/px4_msgs/
  git describe --tags --always
  git branch --show-current
  ```
  By running the installation script, it will automatically match the version.
## File Structure
```
fsc_octorotor_simulation_ros2_px4_gazebo/
├── install_t18_simulation.sh          # Main installation script
├── px4_airframe_configuration/
│   ├── 4012_gz_t18                    # T18 airframe configuration
│   └── README.md                      # README detailing airframe configuration
├── gazebo/
│   ├── gazebo_model/                  # 3D Gazebo models directory
│   │   ├── t18/                       # Main T18
│   │   ├── t18_base/                  # Base T18
│   │   ├── t18_mono_cam/              # T18 with camera
│   │   ├── cad_t18.                   # CAD models of T18
│   │   ├── Pine Tree/                 # Environment object
│   │   └── README.md                  # README detailing gazebo model
│   └── gazebo_world/
│       ├── customforest.sdf           # Custom forest world
│       ├── tree_coordinates.py        # Script for generating random coordinates for trees
│       └── README.md                  # README detailing gazebo world
├── ros2_node/                         # ROS 2 workspace for simulation nodes
│   ├── src/
│   │   ├── px4_msgs/                  # PX4 ROS 2 message definitions (installed via script)
│   │   └── my_offboard_ctrl/          # Custom Python ROS2 package
│   │       ├── package.xml            # ROS2 package metadata
│   │       ├── setup.py               # Python package setup
│   │       ├── setup.cfg              # Package configuration
│   │       ├── resource/              # ROS2 package resource marker
│   │       │   └── my_offboard_ctrl
│   │       ├── my_offboard_ctrl/      # Python source files
│   │       │   ├── __init__.py
│   │       │   ├── gzcam.py           # Gazebo camera handling node
│   │       │   └── offboard_ctrl_example.py # Offboard control node
│   └── README.md                      # README detailing ROS2 node
└── README.md                          # This file
```

## References
This repository builds upon and modifies the ROS2 node from:
- [PX4-ROS2-Gazebo Drone Simulation Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template/tree/main)