# Creating a New Gazebo Model for Tarot T18 Octorotor
## Introduction
A new folder called `t18` was created under `~/PX4-Autopilot/Tools/simulation/gz/models`. In order to create a new gazebo model, the following files must be saved in `~/PX4-Autopilot/Tools/simulation/gz/models/t18`:
- `model.config`
- `model.sdf`
- 3D model (.stl or .dae)

## x500 Model
The simulation was initially created using the `x500_mono_cam` model. The `.sdf` file details are discussed in the following section.

## `x500_mono_cam` `model.sdf` File

### Full Code for `x500_mono_cam` `model.sdf`
The `x500_mono_cam` model is a composite model that combines the base drone `x500` and the monocular camera model `mono_cam`. The following is the code of the `x500_mono_cam` `model.sdf` file:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_mono_cam'>
    <include merge='true'>
      <uri>x500</uri>
    </include>
    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>.12 .03 .242 0 0 0</pose>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>mono_cam/base_link</child>
      <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
    </joint>
  </model>
</sdf>
```
### `x500_mono_cam` `model.sdf` Details
```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_mono_cam'>
```
Defines the new sdf model called `x500_mono_cam`.
```xml
    <include merge='true'>
      <uri>x500</uri>
    </include>
```
Includes the `x500` model as the base of the composite model.
```xml
    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>.12 .03 .242 0 0 0</pose>
    </include>
```
Includes the monocular camera to the model, positioned at (x y z roll pictch yaw) = (.12 .03 .242 0 0 0) relative to origin of `x500`.
```xml
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>mono_cam/base_link</child>
      <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
    </joint>
```
Creates a fixed joint that connects the `base_link` of the `x500` and the `mono_cam`.

## `x500` `model.sdf` File

### Full Code for `x500` `model.sdf`
```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500'>
    <include merge='true'>
      <uri>model://x500_base</uri>
    </include>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_1_joint</jointName>
      <linkName>rotor_1</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>1</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_2_joint</jointName>
      <linkName>rotor_2</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>2</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_3_joint</jointName>
      <linkName>rotor_3</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>3</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
  </model>
</sdf>
```

### `x500` `model.sdf` Details
```xml
    <include merge='true'>
      <uri>model://x500_base</uri>
    </include>
```
Includes the `x500_base` model.
```xml
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
```
Adds motor dynamics for each rotor using motor simulation plugis.\
`<joint name>` - name of the joint in the `x500_base` model \
`<linkname>` - the link connected to the joint in the `x500_base` model \
`<turningDirection>` - motor turning direction (cw or ccw) \
`<timeConstantUp>` - simulates motor acceleration responsiveness. time constant for motor spin-up. \
`<timeConstantDown>`- time constant for spin-down \
`<maxRotVelocity>` - max RPM (rad/s) before clipping \
`<motorConstant>` - maps motor input (PWM or command) to thrust (N/(rad/s)^2) \
`<momentConstant>` - maps motor thrust to torque \
`<commandSubTopic>` - ROS2/Gazebo transport topic where PX4 sends motor commands \
`<motorNumber>` -index of the motor \
`<rotorDragCoefficient>` - drag coefficient for simulating drag force due to spinning rotor \
`<rollingMomentCoefficient>` - rolling coefficient for rolling torques \ 
`<rotorVelocitySlowdownSim>` - scales down rotor velocity to align with simulation. simulates gear ratio \ 
`<motorType>` - specifies the motor control type. `velocity` means PX4 sends angular velocity commands

## `x500_base` `model.sdf` File
### Full Code for `x500_base` `model.sdf`
```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_base'>
    <pose>0 0 .24 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="base_link">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.02166666666666667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.02166666666666667</iyy>
          <iyz>0</iyz>
          <izz>0.04000000000000001</izz>
        </inertia>
      </inertial>
      <gravity>true</gravity>
      <velocity_decay/>
      <visual name="base_link_visual">
        <pose>0 0 .025 0 0 3.141592654</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/NXP-HGD-CF.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_0">
        <pose>0.174 0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_1">
        <pose>-0.174 0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_2">
        <pose>0.174 -0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_3">
        <pose>-0.174 -0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="NXP_FMUK66_FRONT">
        <pose>0.047 .001 .043 1 0 1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.013 .007</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/nxp.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="NXP_FMUK66_TOP">
        <pose>-0.023 0 .0515 0 0 -1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.013 .007</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/nxp.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="RDDRONE_FMUK66_TOP">
        <pose>-.03 0 .0515 0 0 -1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.032 .0034</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/rd.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="base_link_collision_0">
        <pose>0 0 .007 0 0 0</pose>
        <geometry>
          <box>
            <size>0.35355339059327373 0.35355339059327373 0.05</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_1">
        <pose>0 -0.098 -.123 -0.35 0 0</pose>
        <geometry>
          <box>
            <size>0.015 0.015 0.21</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_2">
        <pose>0 0.098 -.123 0.35 0 0</pose>
        <geometry>
          <box>
            <size>0.015 0.015 0.21</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_3">
        <pose>0 -0.132 -.2195 0 0 0</pose>
        <geometry>
          <box>
            <size>0.25 0.015 0.015</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_4">
        <pose>0 0.132 -.2195 0 0 0</pose>
        <geometry>
          <box>
            <size>0.25 0.015 0.015</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure>
          <pressure>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
            </noise>
          </pressure>
        </air_pressure>
      </sensor>
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>
    <link name="rotor_0">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>0.174 -0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_0_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_0_visual_motor_bell">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_0_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_0_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_0</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_1">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>-0.174 0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_1_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_1_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_1_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_1_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_1</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_2">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>0.174 0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_2_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_2_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_2_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_2_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_2</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_3">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>-0.174 -0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_3_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_3_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_3_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_3_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_3</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
  </model>
</sdf>
```

### `x500_base` `model.sdf` Details
```xml
    <pose>0 0 .24 0 0 0</pose>
```
sets te mode's initial position and orientation (x y z roll pitch yaw)\
drone spawns 0.24 m above the grond
```xml
    <self_collide>false</self_collide>
```
prevents the model's own parts from coliding ith each other
```xml
    <static>false</static>
```
makes the model dynamic (i.e. can move and not fixed in place)
```xml
    <link name="base_link">
```
begins definition of main body link of the drone (the central frame structure of the drone)
```xml
      <inertial>
        <mass>2.0</mass>
```
sets the base link mass to 2 kg
```xml
        <inertia>
          <ixx>0.02166666666666667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.02166666666666667</iyy>
          <iyz>0</iyz>
          <izz>0.04000000000000001</izz>
        </inertia>
      </inertial>
```
defines the intertia tensor (how mass is distributed and how the drone rotates about its axes)\
```xml
      <gravity>true</gravity>
```
enables gravity to act of the base link
```xml
      <velocity_decay/>
```
placeholer for velocity decay settings (to simulate air resistance), but not configured
```xml
      <visual name="base_link_visual">
        <pose>0 0 .025 0 0 3.141592654</pose>
```
defines the main visual representation the main visual mesh is positioned at 0.025 m above the base link and 3.14 rad (180 deg) rotated about z-axis.
```xml
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/NXP-HGD-CF.dae</uri>
          </mesh>
        </geometry>
```
loads te mesh for the main body (NXP-HGD-CF.dae) scaled 1:1.
```xml
      </visual>
      <visual name="5010_motor_base_0">
        <pose>0.174 0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_1">
        <pose>-0.174 0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_2">
        <pose>0.174 -0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_3">
        <pose>-0.174 -0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="NXP_FMUK66_FRONT">
        <pose>0.047 .001 .043 1 0 1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.013 .007</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/nxp.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="NXP_FMUK66_TOP">
        <pose>-0.023 0 .0515 0 0 -1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.013 .007</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/nxp.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="RDDRONE_FMUK66_TOP">
        <pose>-.03 0 .0515 0 0 -1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.032 .0034</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/rd.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="base_link_collision_0">
        <pose>0 0 .007 0 0 0</pose>
        <geometry>
          <box>
            <size>0.35355339059327373 0.35355339059327373 0.05</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_1">
        <pose>0 -0.098 -.123 -0.35 0 0</pose>
        <geometry>
          <box>
            <size>0.015 0.015 0.21</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_2">
        <pose>0 0.098 -.123 0.35 0 0</pose>
        <geometry>
          <box>
            <size>0.015 0.015 0.21</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_3">
        <pose>0 -0.132 -.2195 0 0 0</pose>
        <geometry>
          <box>
            <size>0.25 0.015 0.015</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_4">
        <pose>0 0.132 -.2195 0 0 0</pose>
        <geometry>
          <box>
            <size>0.25 0.015 0.015</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure>
          <pressure>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
            </noise>
          </pressure>
        </air_pressure>
      </sensor>
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>
    <link name="rotor_0">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>0.174 -0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_0_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_0_visual_motor_bell">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_0_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_0_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_0</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_1">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>-0.174 0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_1_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_1_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_1_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_1_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_1</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_2">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>0.174 0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_2_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_2_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_2_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_2_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_2</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_3">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>-0.174 -0.174 0.06 0 0 0</pose>
      <inertial>
          <mass>0.016076923076923075</mass>
          <inertia>
            <ixx>3.8464910483993325e-07</ixx>
            <iyy>2.6115851691700804e-05</iyy>
            <izz>2.649858234714004e-05</izz>
          </inertia>
        </inertial>
      <visual name="rotor_3_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_3_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_3_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_3_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_3</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
  </model>
</sdf>
```

### Key Parameters of `x500_base` `model.sdf`
```xml
<model name='x500_base'> <pose>0 0 .24 0 0 0</pose>

<mass>2.0</mass>

<inertia>
  <ixx>0.02166666666666667</ixx>
  <ixy>0</ixy>
  <ixz>0</ixz>
  <iyy>0.02166666666666667</iyy>
  <iyz>0</iyz>
  <izz>0.04000000000000001</izz>
</inertia>

<visual name="base_link_visual">
  <pose>0 0 .025 0 0 3.141592654</pose>
    <mesh>
      <scale>1 1 1</scale>
      <uri>model://x500_base/meshes/NXP-HGD-CF.dae</uri>
    </mesh>

<visual name="5010_motor_base_0">
  <pose>0.174 0.174 .032 0 0 -.45</pose>

<visual name="5010_motor_base_1">
  <pose>-0.174 0.174 .032 0 0 -.45</pose>

<visual name="5010_motor_base_2">
  <pose>0.174 -0.174 .032 0 0 -.45</pose>

<visual name="5010_motor_base_3">
  <pose>-0.174 -0.174 .032 0 0 -.45</pose>

<collision name="base_link_collision_0">
  <pose>0 0 .007 0 0 0</pose>
  <geometry>
    <box>
      <size>0.35355339059327373 0.35355339059327373 0.05</size>
    </box>
  </geometry>

<collision name="base_link_collision_1">
  <pose>0 -0.098 -.123 -0.35 0 0</pose>
  <geometry>
    <box>
      <size>0.015 0.015 0.21</size>
    </box>

<collision name="base_link_collision_2">
  <pose>0 0.098 -.123 0.35 0 0</pose>
  <geometry>
    <box>
      <size>0.015 0.015 0.21</size>
    </box>

<collision name="base_link_collision_3">
  <pose>0 -0.132 -.2195 0 0 0</pose>
  <geometry>
    <box>
      <size>0.25 0.015 0.015</size>
    </box>

<collision name="base_link_collision_4">
  <pose>0 0.132 -.2195 0 0 0</pose>
  <geometry>
    <box>
      <size>0.25 0.015 0.015</size>
    </box>

<link name="rotor_0">
  <pose>0.174 -0.174 0.06 0 0 0</pose>
  <mass>0.016076923076923075</mass>
  <inertia>
    <ixx>3.8464910483993325e-07</ixx>
    <iyy>2.6115851691700804e-05</iyy>
    <izz>2.649858234714004e-05</izz>
  </inertia>

  <visual name="rotor_0_visual">
    <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
      <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
      <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>

  <visual name="rotor_0_visual_motor_bell">
    <pose>0 0 -.032 0 0 0</pose>
      <scale>1 1 1</scale>
      <uri>model://x500_base/meshes/5010Bell.dae</uri>

  <collision name="rotor_0_collision">
    <pose>0 0 0 0 0 0 </pose>
    <geometry>
      <box>
        <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
      </box>

<link name="rotor_1">
  <pose>-0.174 0.174 0.06 0 0 0</pose>
  <mass>0.016076923076923075</mass>
  <inertia>
    <ixx>3.8464910483993325e-07</ixx>
    <iyy>2.6115851691700804e-05</iyy>
    <izz>2.649858234714004e-05</izz>
  </inertia>

  <visual name="rotor_1_visual">
    <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
        <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>

  <visual name="rotor_1_visual_motor_top">
    <pose>0 0 -.032 0 0 0</pose>
        <scale>1 1 1</scale>
        <uri>model://x500_base/meshes/5010Bell.dae</uri>
      </mesh>

  <collision name="rotor_1_collision">
    <pose>0 0 0 0 0 0 </pose>
      <box>
        <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
      </box>


<link name="rotor_2">
  <pose>0.174 0.174 0.06 0 0 0</pose>
      <mass>0.016076923076923075</mass>
      <inertia>
        <ixx>3.8464910483993325e-07</ixx>
        <iyy>2.6115851691700804e-05</iyy>
        <izz>2.649858234714004e-05</izz>
      </inertia>

  <visual name="rotor_2_visual">
    <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
      <mesh>
        <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
        <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
      </mesh>

  <visual name="rotor_2_visual_motor_top">
    <pose>0 0 -.032 0 0 0</pose>
      <mesh>
        <scale>1 1 1</scale>
        <uri>model://x500_base/meshes/5010Bell.dae</uri>
      </mesh>

  <collision name="rotor_2_collision">
    <pose>0 0 0 0 0 0 </pose>
      <box>
        <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
      </box>

<link name="rotor_3">
  <pose>-0.174 -0.174 0.06 0 0 0</pose>
      <mass>0.016076923076923075</mass>
      <inertia>
        <ixx>3.8464910483993325e-07</ixx>
        <iyy>2.6115851691700804e-05</iyy>
        <izz>2.649858234714004e-05</izz>
      </inertia>

  <visual name="rotor_3_visual">
    <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
      <mesh>
        <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
        <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
      </mesh>
    
  <visual name="rotor_3_visual_motor_top">
    <pose>0 0 -.032 0 0 0</pose>
      <mesh>
        <scale>1 1 1</scale>
        <uri>model://x500_base/meshes/5010Bell.dae</uri>
      </mesh>

  <collision name="rotor_3_collision">
    <pose>0 0 0 0 0 0 </pose>
      <box>
        <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
      </box>
``` 
http://googleusercontent.com/immersive_entry_chip/0


---

### T18_base model.sdf key changes

This t18_base.sdf file is a fundamental part of your Gazebo simulation for the Tarot T18 octorotor. Here's a breakdown of the key modifications and why they're important:

Model Name: The model name is set to t18_base, clearly identifying it as the base structure for your new drone.

Inertial Properties (<inertial> in base_link): I've adjusted the mass and inertia values to be significantly higher than those of the x500. The Tarot T18 is a much larger and heavier octorotor, so these changes are crucial for a more realistic physical simulation.

mass: Increased from 2.0 kg (x500) to 8.0 kg (t18).

ixx, iyy, izz: These values, representing the moments of inertia around the X, Y, and Z axes, have been increased to reflect the larger size and mass distribution of the T18. These are initial estimates and may require fine-tuning based on the actual physical specifications of the Tarot T18 for precise simulation.

Base Link Visual Scale: The <scale> for the NXP-HGD-CF.dae mesh in base_link_visual has been increased to 2.0 2.0 1.5. While still using the x500's base mesh for visualization, this scaling makes the drone appear larger in Gazebo, providing a better visual representation of the T18. Ideally, you would replace this with a dedicated 3D mesh of the Tarot T18 if available.

Collision Geometry (<collision> in base_link): The size of the main body collision box and the landing gear collision boxes have been increased. This ensures that the physical interactions within the simulation accurately reflect the larger dimensions of the T18, preventing incorrect collisions or unexpected behavior.

Rotor Links and Joints (Rotor 0-7):

All eight rotor links and their corresponding revolute joints are now explicitly defined. This is a critical change from the quadrotor's four rotors to the octorotor's eight.

Each rotor's <pose> (position and orientation) is set precisely according to the CA_ROTORx_PX and CA_ROTORx_PY parameters from your airframe configuration file. This ensures that the motors are physically placed correctly on the drone model in Gazebo.

The uri for the propeller meshes (1345_prop_ccw.stl or 1345_prop_cw.stl) is chosen based on the CA_ROTORx_KM value from your airframe config:

KM=-0.05 (e.g., CA_ROTOR0_KM) indicates a Counter-Clockwise (CCW) rotation, so 1345_prop_ccw.stl is used.

KM=0.05 (e.g., CA_ROTOR2_KM) indicates a Clockwise (CW) rotation, so 1345_prop_cw.stl is used. This ensures the visual representation of the propellers matches their intended rotation direction.

Sensor Plugins: The essential Gazebo sensor plugins (gz_ros2_imu_plugin, gz_ros2_air_pressure_plugin, gz_ros2_magnetometer_plugin, and gz_ros2_gps_plugin) are included. These plugins are vital for Gazebo to publish simulated sensor data (IMU, barometer, magnetometer, GPS) that the PX4 firmware will use for its flight control algorithms. The <ros><namespace> for these plugins is set to t18, aligning with your PX4 airframe configuration.

Next Steps
To get your Tarot T18 octorotor simulation running, you'll need to:

Save this file as t18_base.sdf. It's recommended to place it in a new directory within your Gazebo models path, for example, PX4-Autopilot/Tools/simulation/gz/models/t18_base.

Ensure Mesh Files are Present: Make sure all referenced mesh files (e.g., NXP-HGD-CF.dae, 5010Base.dae, 5010Bell.dae, 1345_prop_ccw.stl, 1345_prop_cw.stl, and the texture images nxp.png, rd.png) are correctly located within the x500_base model directory or a similar path that Gazebo can resolve. If you don't have these specific files, you might need to copy them from the original x500_base model's asset folders.

Use t18.sdf: Remember that this t18_base.sdf file is designed to be included by the main t18.sdf file (which you previously reviewed in the Canvas). The t18.sdf file is where the gz-sim-multicopter-motor-model-system plugins are defined for each motor.

Update ROS2 Launch: Finally, ensure your ROS2 launch file is updated to spawn the t18 model (which in turn includes t18_base) and that the PX4_SIM_MODEL environment variable is set to t18 when launching the PX4 firmware.






Key Components of t18.sdf
This t18.sdf Canvas now defines the complete Gazebo model for your Tarot T18 octorotor by combining the structural elements from t18_base.sdf with the dynamic motor behaviors.

Model Inclusion: The <include merge='true'> tag is used to pull in all the links, visuals, collisions, and sensor definitions from your t18_base.sdf file. This modular approach keeps your SDF files organized.

Multicopter Motor Model Plugins: Eight instances of the gz-sim-multicopter-motor-model-system plugin are included, one for each rotor. These plugins are essential for simulating the realistic behavior of each motor in Gazebo.

jointName and linkName: Each plugin is explicitly linked to its corresponding rotor joint and link (e.g., rotor_0_joint, rotor_0), which are defined in t18_base.sdf.

turningDirection: This parameter is crucial for simulating the correct rotational torque and thrust. It's set to ccw (counter-clockwise) or cw (clockwise) for each motor, directly corresponding to the KM (motor direction) values defined in your PX4 airframe configuration file for the T18.

motorNumber: Each motor is assigned a unique number (0 through 7), which PX4 uses to send individual commands to each propeller.

commandSubTopic: This is set to command/motor_speed, which is the standard topic Gazebo listens to for motor commands from PX4.

Motor Parameters: Parameters like timeConstantUp, timeConstantDown, maxRotVelocity, motorConstant, momentConstant, rotorDragCoefficient, and rollingMomentCoefficient are included. These values, initially derived from the x500 model, define the motor's dynamic response and the thrust/torque it generates. For the most accurate simulation, you might need to tune these parameters to match the real-world specifications of the Tarot T18's motors and propellers.

Next Steps
To use this t18.sdf file in your simulation:

Save this file as t18.sdf in a new directory, for example, PX4-Autopilot/Tools/simulation/gz/models/t18.

Ensure that your t18_base.sdf file is correctly located (e.g., in PX4-Autopilot/Tools/simulation/gz/models/t18_base).

Finally, update your ROS2 launch file to spawn the t18 model. This will typically involve setting a launch argument like model: 't18' or similar, depending on your specific launch setup.





Key Changes and How to Use It
This t18_mono_cam.sdf file is designed to be a composite model in Gazebo, combining your newly defined T18 octorotor with a monocular camera.

Model Name: The top-level model name is now t18_mono_cam, clearly indicating the drone and its attached camera.

Drone Model Inclusion: The <include> tag now references model://t18. This means Gazebo will load all the components and plugins defined in your t18.sdf file (which itself includes t18_base.sdf and the motor plugins).

Camera Inclusion: The <include> tag for model://mono_cam remains the same, as it's a generic camera model. The <pose> defines its offset and orientation relative to the drone's origin.

Fixed Joint: The CameraJoint is a fixed type, which means the camera will not move independently but will be rigidly attached to the drone's base_link. This is standard for onboard cameras.

Next Steps:
Save the file: Save the content above as t18_mono_cam.sdf. It's recommended to place it in a new directory within your Gazebo models path, for example, PX4-Autopilot/Tools/simulation/gz/models/t18_mono_cam.

Ensure t18.sdf is accessible: Make sure the t18.sdf file (which you previously created) is located in PX4-Autopilot/Tools/simulation/gz/models/t18.

Ensure mono_cam model exists: Verify that the mono_cam model (including its SDF file and any meshes/textures) is available in your Gazebo model paths. If not, you might need to copy it from your existing x500_mono_cam setup or a Gazebo default model library.

Update your ROS2 launch file: Finally, modify your ROS2 launch file to spawn the t18_mono_cam model instead of t18 or x500.

With these files in place, you should be able to launch your simulation with the Tarot T18 octorotor equipped with a monocular camera.