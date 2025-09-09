# Details of Airframe Configuration File
Covers the details of aircraft configuration file based on the `4001_gz_x500` and `4010_gz_x500_mono_cam` files found in `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`. `4010_gz_x500_mono_cam` is a speccial configuraiton of `4001_gz_x500` where a monocular camera model is attached to the `4001_gz_x500` airframe model in gazebo.



## Background
As part of creating the simulation for the T18 octorotor, I initially was using the x500 quadrotor, a pre-built quadrotor model. The code I was running to initiate the simulation was:
1. Initiate Micro XRCE Agent
```bash
cd ~/px4_ros_uxrce_dds_ws/ 
source /opt/ros/humble/setup.bash
source install/local_setup.bash
MicroXRCEAgent udp4 -p 8888
```


2. Initiate PX4 and Start Simulation in Gazebo, where the following calls for `4010` airframe configuration
```bash
cd ~/PX4-Autopilot/
PX4_SYS_AUTOSTART=4010 \
PX4_SIM_MODEL=gz_x500_mono_cam \
PX4_GZ_MODEL_POSE="1,1,0.1,0,0,0.9" \
PX4_GZ_WORLD=customforest \
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```


3. Initiate ROS2 Node
```bash
cd ~/PX4-ROS2-Gazebo-Drone-Simulation-Template/ws_ros2
colcon buildg
source install/local_setup.bash
ros2 run my_offboard_ctrl offboard_ctrl_example
```



## Shell Script File: `4010_gz_x500_mono_cam`
When running
```bash
PX4_SYS_AUTOSTART=4010 \
```
The airframe configuration file `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4010_gz_x500_mono_cam` is called.
The full shell script file is shown below:
```sh
#!/bin/sh
#
# @name Gazebo x500 mono cam
#
# @type Quadrotor
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_mono_cam}

. ${R}etc/init.d-posix/airframes/4001_gz_x500
```


## Details of `4010_gz_x500_mono_cam`
```sh
#!/bin/sh
```
Shell interpreter. Indicates this is a shell script and should be interpreted by the `sh` shell.
```sh
# @name Gazebo x500 mono cam
#
# @type Quadrotor
```
Human-readable name (shown in QGC) and type.
```sh
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_mono_cam}
```
Sets x500_mono_cam as the gazebo model to use if not specified.
```sh
. ${R}etc/init.d-posix/airframes/4001_gz_x500
```
Imports the `4001_gz_x500` as the airframe configuration file to use. 
Essentially, the `4010` airframe configuration is the same as the `4001` airframe configuraiton, but uses a different gazebo model (namely, `x500_mono_cam` instead of `x500`).


## Full Code for `4001_gz_x500`
```sh
#!/bin/sh
#
# @name Gazebo x500
#
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500}

param set-default SIM_GZ_EN 1

param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 0
param set-default SENS_EN_MAGSIM 1

param set-default CA_AIRFRAME 0

param set-default CA_ROTOR_COUNT 4

param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

param set-default MPC_THR_HOVER 0.60
```
## Details of `4001_gz_x500`
```sh
#!/bin/sh
#
# @name Gazebo x500
#
# @type Quadrotor
#
```
Shell script decleration and sets name and type.
```sh
. ${R}etc/init.d/rc.mc_defaults

```
Source PX4 default multicoptor control parameters.
```sh
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500}
```
Set simulator to `gz` (gazebo), gazebo world to `default.sdf`, and gazebo model to `x500`, if not specified.
```sh
param set-default SIM_GZ_EN 1
```
Enable gazebo simulation mode. Activates PX4's Gazebo bridge for simulated vehicle communication.
```sh 
param set-default SENS_EN_GPSSIM 1
```
Enable GPS simulation. Use simulated GPS data.
```ah
param set-default SENS_EN_BAROSIM 0
```
Disables barometer. Use real barometer.
```sh
param set-default SENS_EN_MAGSIM 1
```
Enables magnetometer simulation. Use simulated magnetometer data.
```sh
param set-default CA_AIRFRAME 0
```
Airframe selection. Defines which mixer implementation to use. Selected `0` (Multirotor). ([Control Allocation (Mixing)](https://docs.px4.io/main/en/concept/control_allocation), [CA_AIRFRAME](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#CA_AIRFRAME)) 
```sh
param set-default CA_ROTOR_COUNT 4
```
Set the number of rotors to 4 (quadrotor)
```sh
param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05
```
`param set-default CA_ROTOR0_PX` sets x position of rotor 0 in metres.\
`param set-default CA_ROTOR0_PY` sets y position of rotor 0 in metres.\
`param set-default CA_ROTOR0_KM` sets moment coefficient of rotor 0. If not specified, a default value of 0.05 is used. +ve for CCW and -ve for CW. Moment coefficient is defined as Torque = KM * Thrust. ([model.yaml](https://github.com/beniaminopozzan/PX4-Autopilot/blob/9b7a8d45685ead479abeed76f8faff61f5832ac4/src/modules/control_allocator/module.yaml#L207-L221))
```sh
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104
```
Links PX4 motor outputs to Gazebo motor joints. Assigns each motor to an ESC output.
```sh
param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000
```
Sets ESC min/max PWM values in simulation. PWM signal range for motor control. 
Min (150 μs): ESC idle/stop signal.
Max (1000 μs): ESC full power signal.
```sh
param set-default MPC_THR_HOVER 0.60
```
Tells the position controller (MPC) that 60% throttle is needed to hover based on weight-to-power ratio of x500.

## Airframe Configuration File for T18 Octorotor
```sh
#!/bin/sh
#
# @name T18 Octorotor
#
# @type Octorotor x
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=customforest}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=t18}

param set-default SIM_GZ_EN 1

# set everything to simulated data
param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 1
param set-default SENS_EN_MAGSIM 1

# MAVLink vehicle type for octorotor
param set-default MAV_TYPE 14

param set-default CA_AIRFRAME 0

param set-default CA_ROTOR_COUNT 8

param set-default CA_ROTOR0_PX 0.29
param set-default CA_ROTOR0_PY 0.12
param set-default CA_ROTOR0_KM -0.05

param set-default CA_ROTOR1_PX -0.29
param set-default CA_ROTOR1_PY -0.12
param set-default CA_ROTOR1_KM -0.05

param set-default CA_ROTOR2_PX 0.12
param set-default CA_ROTOR2_PY 0.29
param set-default CA_ROTOR2_KM 0.05

param set-default CA_ROTOR3_PX -0.29
param set-default CA_ROTOR3_PY 0.12
param set-default CA_ROTOR3_KM 0.05

param set-default CA_ROTOR4_PX 0.29
param set-default CA_ROTOR4_PY -0.12
param set-default CA_ROTOR4_KM 0.05

param set-default CA_ROTOR5_PX -0.12
param set-default CA_ROTOR5_PY -0.29
param set-default CA_ROTOR5_KM 0.05

param set-default CA_ROTOR6_PX 0.12
param set-default CA_ROTOR6_PY -0.29
param set-default CA_ROTOR6_KM -0.05

param set-default CA_ROTOR7_PX -0.12
param set-default CA_ROTOR7_PY 0.29
param set-default CA_ROTOR7_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104
param set-default SIM_GZ_EC_FUNC5 105
param set-default SIM_GZ_EC_FUNC6 106
param set-default SIM_GZ_EC_FUNC7 107
param set-default SIM_GZ_EC_FUNC8 108

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150
param set-default SIM_GZ_EC_MIN5 150
param set-default SIM_GZ_EC_MIN6 150
param set-default SIM_GZ_EC_MIN7 150
param set-default SIM_GZ_EC_MIN8 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000
param set-default SIM_GZ_EC_MAX5 1000
param set-default SIM_GZ_EC_MAX6 1000
param set-default SIM_GZ_EC_MAX7 1000
param set-default SIM_GZ_EC_MAX8 1000

# lower hover throttle due to 8 motors providing more thrust
param set-default MPC_THR_HOVER 0.35

# Optional PID parameters
# Simulation-optimized control gains (can be more aggressive than real hardware)
# param set-default MC_PITCHRATE_P 0.18
# param set-default MC_ROLLRATE_P 0.18
# param set-default MC_YAWRATE_P 0.30

# Higher control bandwidth suitable for simulation
# param set-default MC_PITCH_P 9.0
# param set-default MC_ROLL_P 9.0
# param set-default MC_YAW_P 4.0
```

## Documentations
- [PX4 Gazebo Simulation Documentation](https://docs.px4.io/main/en/sim_gazebo_gz/)
  - Shows the steps for adding a new model
- [Adding a Frame Configuration](https://docs.px4.io/main/en/dev_airframes/adding_a_new_frame.html)
- [Airframes Reference](https://docs.px4.io/main/en/airframes/airframe_reference.html#copter_quadrotor_x_generic_quadcopter)
- [~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix
/airframes/](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d-posix/airframes)
- [Control Allocation (Mixing)](https://docs.px4.io/main/en/concept/control_allocation)
- [Simulation-In-Hardware (SIH)](https://docs.px4.io/main/en/sim_sih/)