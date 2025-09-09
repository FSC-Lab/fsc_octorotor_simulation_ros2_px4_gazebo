#!/usr/bin/env python3

"""
Original file from: https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template
License: BSD 3-Clause License
Modified to include waypoint following functionality.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import math
import time

# Camera module
import cv2
from .gzcam import GzCam
import threading


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode with waypoint following."""

    def __init__(self) -> None:
        super().__init__('offboard_control_waypoint_following')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.5
        
        # Waypoint definition
        self.waypoints = [
            [0.0, 0.0, -1.5],
            [-2.0, 2.0, -1.5],
            [-4.0, 0.0, -1.5],
            [-8.0, 0.0, -1.5],
            [-8.0, 8.0, -1.5],
        ]
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.2  # meters
        self.waypoint_reached = False
        self.mission_complete = False
        
        # States for the mission
        self.TAKEOFF = 0
        self.WAYPOINT_FOLLOWING = 1
        self.LANDING = 2
        self.mission_state = self.TAKEOFF
        
        # Timing
        self.waypoint_hold_time = 0.5  # seconds to hold at each waypoint
        self.waypoint_arrival_time = None

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def add_waypoint(self, x: float, y: float, z: float):
        """Add a waypoint to the mission."""
        self.waypoints.append([x, y, z])
        self.get_logger().info(f'Added waypoint: [{x}, {y}, {z}]')

    def set_waypoints(self, waypoints: list):
        """Set a list of waypoints for the mission."""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.get_logger().info(f'Set {len(waypoints)} waypoints for mission')

    def get_distance_to_waypoint(self, waypoint):
        """Calculate 3D distance to a waypoint."""
        dx = self.vehicle_local_position.x - waypoint[0]
        dy = self.vehicle_local_position.y - waypoint[1]
        dz = self.vehicle_local_position.z - waypoint[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def is_waypoint_reached(self, waypoint):
        """Check if current waypoint is reached within tolerance."""
        return self.get_distance_to_waypoint(waypoint) < self.waypoint_tolerance

    def get_current_waypoint(self):
        """Get the current target waypoint."""
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

    def advance_waypoint(self):
        """Move to the next waypoint."""
        self.current_waypoint_index += 1
        self.waypoint_reached = False
        self.waypoint_arrival_time = None
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.mission_complete = True
            self.get_logger().info('All waypoints completed!')
        else:
            waypoint = self.get_current_waypoint()
            self.get_logger().info(f'Advancing to waypoint {self.current_waypoint_index}: {waypoint}')

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # Only log occasionally to avoid spam
        if self.offboard_setpoint_counter % 50 == 0:  # Log every 5 seconds
            self.get_logger().info(f"Publishing position setpoint: [{x:.2f}, {y:.2f}, {z:.2f}]")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # Initial setup - engage offboard mode and arm
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # State machine for mission execution
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            if self.mission_state == self.TAKEOFF:
                # Takeoff phase
                if self.vehicle_local_position.z > self.takeoff_height:
                    # Still ascending
                    self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                else:
                    # Takeoff complete, switch to waypoint following
                    self.mission_state = self.WAYPOINT_FOLLOWING
                    self.get_logger().info('Takeoff complete, starting waypoint following')
            
            elif self.mission_state == self.WAYPOINT_FOLLOWING:
                # Waypoint following phase
                if not self.mission_complete:
                    current_waypoint = self.get_current_waypoint()
                    
                    if current_waypoint is not None:
                        # Publish setpoint for current waypoint
                        self.publish_position_setpoint(
                            current_waypoint[0], 
                            current_waypoint[1], 
                            current_waypoint[2]
                        )
                        
                        # Check if waypoint is reached
                        if self.is_waypoint_reached(current_waypoint):
                            if not self.waypoint_reached:
                                # Just arrived at waypoint
                                self.waypoint_reached = True
                                self.waypoint_arrival_time = time.time()
                                distance = self.get_distance_to_waypoint(current_waypoint)
                                self.get_logger().info(
                                    f'Reached waypoint {self.current_waypoint_index}: '
                                    f'{current_waypoint} (distance: {distance:.3f}m)'
                                )
                            
                            # Check if we've held position long enough
                            if (time.time() - self.waypoint_arrival_time) > self.waypoint_hold_time:
                                self.advance_waypoint()
                
                else:
                    # Mission complete, switch to landing
                    self.mission_state = self.LANDING
                    self.get_logger().info('Mission complete, initiating landing')
            
            elif self.mission_state == self.LANDING:
                # Landing phase
                self.land()
                exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


# Camera module
def launch_cam_receiver():
    cam = GzCam("/camera", (640, 480))
    while True:
        img = cam.get_next_image()
        cv2.imshow('pic-display', img)
        cv2.waitKey(1)


def main(args=None) -> None:
    print('Starting camera...')
    cam_thread = threading.Thread(target=launch_cam_receiver)
    cam_thread.daemon = True  # Make thread daemon so it exits when main exits
    cam_thread.start()

    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)