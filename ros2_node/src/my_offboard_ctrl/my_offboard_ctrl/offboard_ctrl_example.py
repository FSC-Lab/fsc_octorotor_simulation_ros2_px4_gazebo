#!/usr/bin/env python3

"""
Original file from: https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template
License: BSD 3-Clause License
Modified to include waypoint following functionality, status updates, and 3D visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import math
import time
import threading

# Camera module
import cv2
from .gzcam import GzCam

# 3D Plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


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
        
        # Key waypoint definition (user-defined key locations)
        self.key_waypoints = [
            [0.0, 0.0, -1.5],
            [-2.0, 2.0, -1.5],
            [-4.0, 0.0, -1.5],
            [-8.0, 0.0, -1.5],
            [-8.0, 8.0, -1.5],
        ]
        
        # Generate dense waypoint trajectory for smooth flight
        self.waypoints = self.generate_smooth_trajectory(self.key_waypoints)
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints from {len(self.key_waypoints)} key locations')
        
        # Trajectory following parameters
        self.current_waypoint_index = 0
        self.trajectory_advance_distance = 0.5  # meters - how close to advance to next waypoint
        self.lookahead_distance = 1.0  # meters - look ahead for smoother tracking
        self.mission_complete = False
        
        # States for the mission
        self.TAKEOFF = 0
        self.TRAJECTORY_FOLLOWING = 1  # Changed from WAYPOINT_FOLLOWING
        self.LANDING = 2
        self.mission_state = self.TAKEOFF
        
        # Trajectory timing
        self.trajectory_start_time = None

        # Status update timer (every 2 seconds)
        self.status_counter = 0
        self.status_update_interval = 20  # Every 2 seconds (0.1 * 20)
        
        # Position tracking for 3D plot
        self.position_history = []
        self.start_time = time.time()
        
        # 3D Plot setup
        self.setup_3d_plot()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_smooth_trajectory(self, key_waypoints, waypoint_spacing=0.3):
        """Generate a dense, smooth trajectory using B-spline-like interpolation."""
        if len(key_waypoints) < 2:
            return key_waypoints.copy()
        
        # Convert to numpy for easier manipulation
        key_points = np.array(key_waypoints)
        
        # Create parameter values for each key waypoint (0 to 1)
        t_key = np.linspace(0, 1, len(key_points))
        
        # Create dense parameter array for interpolation
        # Calculate total path length to determine number of points needed
        total_length = 0
        for i in range(len(key_points) - 1):
            total_length += np.linalg.norm(key_points[i+1] - key_points[i])
        
        # Calculate number of waypoints needed based on desired spacing
        num_waypoints = max(int(total_length / waypoint_spacing), len(key_points) * 5)
        t_dense = np.linspace(0, 1, num_waypoints)
        
        # Interpolate each dimension separately using cubic interpolation
        from scipy.interpolate import interp1d
        
        try:
            # Use cubic spline interpolation for smooth curves
            interp_x = interp1d(t_key, key_points[:, 0], kind='cubic', 
                               bounds_error=False, fill_value='extrapolate')
            interp_y = interp1d(t_key, key_points[:, 1], kind='cubic', 
                               bounds_error=False, fill_value='extrapolate')
            interp_z = interp1d(t_key, key_points[:, 2], kind='cubic', 
                               bounds_error=False, fill_value='extrapolate')
            
            # Generate smooth trajectory points
            smooth_x = interp_x(t_dense)
            smooth_y = interp_y(t_dense)
            smooth_z = interp_z(t_dense)
            
            # Combine into waypoint list
            smooth_waypoints = np.column_stack((smooth_x, smooth_y, smooth_z)).tolist()
            
        except ImportError:
            # Fallback to linear interpolation if scipy not available
            self.get_logger().warn("scipy not available, using linear interpolation. Install scipy for smoother trajectories.")
            smooth_waypoints = []
            
            for i in range(len(t_dense)):
                # Find which segment we're in
                t = t_dense[i]
                if t >= 1.0:
                    smooth_waypoints.append(key_points[-1].tolist())
                    continue
                    
                # Find the segment
                segment_idx = min(int(t * (len(key_points) - 1)), len(key_points) - 2)
                local_t = (t * (len(key_points) - 1)) - segment_idx
                
                # Linear interpolation
                start_point = key_points[segment_idx]
                end_point = key_points[segment_idx + 1]
                interpolated_point = start_point + local_t * (end_point - start_point)
                smooth_waypoints.append(interpolated_point.tolist())
        
        self.get_logger().info(f'Generated {len(smooth_waypoints)} trajectory points with {waypoint_spacing:.2f}m spacing')
        self.get_logger().info(f'Total planned trajectory length: ~{total_length:.2f}m')
        
        return smooth_waypoints

    def get_current_trajectory_point(self):
        """Get the current target point on the trajectory."""
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

    def get_lookahead_point(self):
        """Get a lookahead point for smoother trajectory following."""
        if self.current_waypoint_index >= len(self.waypoints):
            return None
            
        # Calculate how many points to look ahead based on current speed and lookahead distance
        lookahead_points = max(1, int(self.lookahead_distance / 0.3))  # Assuming 0.3m spacing
        lookahead_index = min(self.current_waypoint_index + lookahead_points, len(self.waypoints) - 1)
        
        return self.waypoints[lookahead_index]

    def advance_trajectory_point(self):
        """Advance to the next point on the trajectory."""
        current_point = self.get_current_trajectory_point()
        if current_point is None:
            return
            
        # Check if we're close enough to the current point to advance
        distance = self.get_distance_to_waypoint(current_point)
        
        if distance < self.trajectory_advance_distance:
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints):
                self.mission_complete = True
                self.get_logger().info('Trajectory following complete!')
            else:
                # Only log occasionally to avoid spam
                if self.current_waypoint_index % 20 == 0:  # Every 20 points
                    progress = (self.current_waypoint_index / len(self.waypoints)) * 100
                    self.get_logger().info(f'Trajectory progress: {progress:.1f}% ({self.current_waypoint_index}/{len(self.waypoints)})')

    def generate_intermediate_waypoints(self, key_waypoints, max_segment_length=2.0):
        """Legacy function - kept for compatibility but now calls generate_smooth_trajectory."""
        return self.generate_smooth_trajectory(key_waypoints, waypoint_spacing=max_segment_length/3)

    def get_planned_path(self):
        """Get the complete planned path including takeoff and all waypoints."""
        planned_path = [[0.0, 0.0, 0.0]]  # Start at ground level
        planned_path.append([0.0, 0.0, -self.takeoff_height])  # Takeoff position
        
        # Add all waypoints
        for waypoint in self.waypoints:
            planned_path.append(waypoint)
            
        return planned_path
    def setup_3d_plot(self):
        """Setup the 3D plot for visualization."""
        plt.ion()  # Turn on interactive mode
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Plot key waypoints (larger, red)
        key_waypoints_array = np.array(self.key_waypoints)
        self.ax.scatter(key_waypoints_array[:, 0], key_waypoints_array[:, 1], -key_waypoints_array[:, 2], 
                       c='red', marker='s', s=150, label='Key Waypoints', edgecolors='black', linewidth=2)
        
        # Add key waypoint numbers
        for i, waypoint in enumerate(self.key_waypoints):
            self.ax.text(waypoint[0], waypoint[1], -waypoint[2] + 0.1, f'KWP{i}', 
                        fontsize=10, fontweight='bold', color='red')
        
        # Plot all waypoints (including intermediate ones, smaller, orange)
        waypoints_array = np.array(self.waypoints)
        intermediate_waypoints = []
        for i, wp in enumerate(self.waypoints):
            if wp not in self.key_waypoints:
                intermediate_waypoints.append(wp)
        
        if intermediate_waypoints:
            intermediate_array = np.array(intermediate_waypoints)
            self.ax.scatter(intermediate_array[:, 0], intermediate_array[:, 1], -intermediate_array[:, 2], 
                           c='orange', marker='o', s=50, label='Intermediate Waypoints', alpha=0.7)
        
        # Plot planned path (green dashed line)
        planned_path = self.get_planned_path()
        planned_array = np.array(planned_path)
        self.ax.plot(planned_array[:, 0], planned_array[:, 1], -planned_array[:, 2], 
                    'g--', linewidth=2, alpha=0.8, label='Planned Path')
        
        # Plot takeoff position
        self.ax.scatter([0], [0], [0], c='green', marker='^', s=200, label='Takeoff', 
                       edgecolors='black', linewidth=2)
        
        # Setup plot appearance
        self.ax.set_xlabel('X (North) [m]')
        self.ax.set_ylabel('Y (East) [m]')
        self.ax.set_zlabel('Z (Up) [m]')
        self.ax.set_title('Drone Flight Path - Real-time Tracking\n(Key Waypoints, Planned Path, and Actual Flight)')
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.ax.grid(True)
        
        # Initialize empty line for actual trajectory
        self.trajectory_line, = self.ax.plot([], [], [], 'b-', linewidth=3, label='Actual Flight Path')
        self.current_pos_point = self.ax.scatter([], [], [], c='blue', marker='o', s=80, 
                                               label='Current Position', edgecolors='white', linewidth=1)
        
        # Set equal aspect ratio and adjust view
        max_range = max(
            np.max(np.abs(planned_array[:, 0])),
            np.max(np.abs(planned_array[:, 1])),
            np.max(np.abs(planned_array[:, 2]))
        )
        self.ax.set_xlim([-max_range*1.1, max_range*1.1])
        self.ax.set_ylim([-max_range*1.1, max_range*1.1])
        self.ax.set_zlim([0, max_range*1.1])
        
        plt.tight_layout()
        plt.show(block=False)

    def update_3d_plot(self):
        """Update the 3D plot with current position."""
        if hasattr(self.vehicle_local_position, 'x'):
            # Add current position to history
            current_pos = [
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                -self.vehicle_local_position.z  # Flip Z for visualization (up is positive)
            ]
            self.position_history.append(current_pos)
            
            # Update trajectory line
            if len(self.position_history) > 1:
                history_array = np.array(self.position_history)
                self.trajectory_line.set_data_3d(
                    history_array[:, 0],
                    history_array[:, 1],
                    history_array[:, 2]
                )
            
            # Update current position point
            self.current_pos_point._offsets3d = ([current_pos[0]], [current_pos[1]], [current_pos[2]])
            
            # Refresh the plot
            try:
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
            except:
                pass  # Handle any GUI-related errors silently

    def print_status_update(self):
        """Print comprehensive status update."""
        print("\n" + "="*60)
        print(f"DRONE STATUS UPDATE - Time: {time.time() - self.start_time:.1f}s")
        print("="*60)
        
        # Current position
        print(f"Current Position: X={self.vehicle_local_position.x:.2f}, "
              f"Y={self.vehicle_local_position.y:.2f}, Z={self.vehicle_local_position.z:.2f}")
        
        # Mission state
        state_names = {self.TAKEOFF: "TAKEOFF", 
                      self.TRAJECTORY_FOLLOWING: "TRAJECTORY_FOLLOWING", 
                      self.LANDING: "LANDING"}
        print(f"Mission State: {state_names.get(self.mission_state, 'UNKNOWN')}")
        
        # Navigation state
        nav_state_names = {
            VehicleStatus.NAVIGATION_STATE_MANUAL: "MANUAL",
            VehicleStatus.NAVIGATION_STATE_ALTCTL: "ALTITUDE_CONTROL",
            VehicleStatus.NAVIGATION_STATE_POSCTL: "POSITION_CONTROL",
            VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: "AUTO_MISSION",
            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "AUTO_LOITER",
            VehicleStatus.NAVIGATION_STATE_AUTO_RTL: "AUTO_RTL",
            VehicleStatus.NAVIGATION_STATE_OFFBOARD: "OFFBOARD",
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "AUTO_LAND",
            VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: "AUTO_TAKEOFF"
        }
        nav_state = nav_state_names.get(self.vehicle_status.nav_state, f"UNKNOWN({self.vehicle_status.nav_state})")
        print(f"Navigation State: {nav_state}")
        
        # Armed status
        arm_status = "ARMED" if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED else "DISARMED"
        print(f"Arm Status: {arm_status}")
        
        # Trajectory information
        if self.mission_state == self.TRAJECTORY_FOLLOWING:
            if not self.mission_complete:
                current_point = self.get_current_trajectory_point()
                if current_point:
                    distance = self.get_distance_to_waypoint(current_point)
                    progress = (self.current_waypoint_index / len(self.waypoints)) * 100
                    
                    print(f"Current Trajectory Point: [{current_point[0]:.2f}, {current_point[1]:.2f}, {current_point[2]:.2f}]")
                    print(f"Distance to Point: {distance:.3f}m")
                    print(f"Trajectory Progress: {progress:.1f}% ({self.current_waypoint_index}/{len(self.waypoints)})")
                    
                    # Show lookahead point
                    lookahead = self.get_lookahead_point()
                    if lookahead and lookahead != current_point:
                        print(f"Lookahead Point: [{lookahead[0]:.2f}, {lookahead[1]:.2f}, {lookahead[2]:.2f}]")
                    
                    # Calculate estimated time remaining
                    if self.trajectory_start_time and self.current_waypoint_index > 0:
                        elapsed_time = time.time() - self.trajectory_start_time
                        points_per_second = self.current_waypoint_index / elapsed_time
                        remaining_points = len(self.waypoints) - self.current_waypoint_index
                        est_time_remaining = remaining_points / points_per_second if points_per_second > 0 else 0
                        print(f"Estimated Time Remaining: {est_time_remaining:.1f}s")
            else:
                print("Trajectory following completed!")
        
        elif self.mission_state == self.TAKEOFF:
            altitude_error = abs(self.vehicle_local_position.z - self.takeoff_height)
            print(f"Taking off... Target altitude: {self.takeoff_height:.2f}m, Error: {altitude_error:.3f}m")
        
        print("="*60)

    def set_waypoints_from_file(self, waypoints_list):
        """Set waypoints from an external source (e.g., B-spline generated points)."""
        self.waypoints = waypoints_list
        self.key_waypoints = waypoints_list  # Treat all as key waypoints if set externally
        self.current_waypoint_index = 0
        self.mission_complete = False
        self.get_logger().info(f'Loaded {len(waypoints_list)} waypoints from external source')
        
        # Update 3D plot if it exists
        if hasattr(self, 'fig'):
            self.setup_3d_plot()

    def set_trajectory_parameters(self, advance_distance=0.3, lookahead_distance=1.0, waypoint_spacing=0.2):
        """Configure trajectory following parameters."""
        self.trajectory_advance_distance = advance_distance
        self.lookahead_distance = lookahead_distance
        
        # Regenerate trajectory with new spacing if using generated waypoints
        if hasattr(self, 'key_waypoints'):
            self.waypoints = self.generate_smooth_trajectory(self.key_waypoints, waypoint_spacing)
            self.get_logger().info(f'Regenerated trajectory with {waypoint_spacing:.2f}m spacing')
            
        self.get_logger().info(f'Updated trajectory parameters: advance={advance_distance:.2f}m, lookahead={lookahead_distance:.2f}m')
    def add_waypoint(self, x: float, y: float, z: float):
        """Add a waypoint to the mission."""
        self.key_waypoints.append([x, y, z])
        # Regenerate smooth trajectory
        self.waypoints = self.generate_smooth_trajectory(self.key_waypoints)
        self.get_logger().info(f'Added waypoint: [{x}, {y}, {z}], regenerated trajectory with {len(self.waypoints)} points')

    def set_waypoints(self, waypoints: list):
        """Set a list of waypoints for the mission."""
        self.key_waypoints = waypoints
        self.waypoints = self.generate_smooth_trajectory(self.key_waypoints)
        self.current_waypoint_index = 0
        self.get_logger().info(f'Set {len(waypoints)} key waypoints, generated {len(self.waypoints)} trajectory points')

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
        """Legacy function - kept for compatibility."""
        return self.get_current_trajectory_point()

    def advance_waypoint(self):
        """Legacy function - kept for compatibility."""
        self.advance_trajectory_point()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        
        # Update 3D plot periodically (every 5 callbacks to reduce CPU load)
        if self.offboard_setpoint_counter % 5 == 0:
            self.update_3d_plot()

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

        # Periodic status updates (every 2 seconds)
        if self.status_counter >= self.status_update_interval:
            self.print_status_update()
            self.status_counter = 0
        else:
            self.status_counter += 1

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
                    # Takeoff complete, switch to trajectory following
                    self.mission_state = self.TRAJECTORY_FOLLOWING
                    self.trajectory_start_time = time.time()
                    self.get_logger().info('Takeoff complete, starting smooth trajectory following')
            
            elif self.mission_state == self.TRAJECTORY_FOLLOWING:
                # Smooth trajectory following phase
                if not self.mission_complete:
                    # Get current target point and lookahead point for smoother control
                    current_point = self.get_current_trajectory_point()
                    lookahead_point = self.get_lookahead_point()
                    
                    if current_point is not None:
                        # Use lookahead point if available for smoother trajectory
                        target_point = lookahead_point if lookahead_point else current_point
                        
                        # Publish setpoint for target point
                        self.publish_position_setpoint(
                            target_point[0], 
                            target_point[1], 
                            target_point[2]
                        )
                        
                        # Continuously advance through trajectory points
                        self.advance_trajectory_point()
                
                else:
                    # Trajectory complete, switch to landing
                    self.mission_state = self.LANDING
                    self.get_logger().info('Smooth trajectory complete, initiating landing')
            
            elif self.mission_state == self.LANDING:
                # Landing phase
                self.land()
                # Create final comprehensive plot and save
                self.create_final_flight_plot()
                exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def create_final_flight_plot(self):
        """Create and save a comprehensive final flight plot."""
        plt.ioff()  # Turn off interactive mode for final plot
        
        # Create a new figure for the final plot
        final_fig = plt.figure(figsize=(16, 12))
        final_ax = final_fig.add_subplot(111, projection='3d')
        
        # Plot key waypoints (larger, red squares)
        key_waypoints_array = np.array(self.key_waypoints)
        final_ax.scatter(key_waypoints_array[:, 0], key_waypoints_array[:, 1], -key_waypoints_array[:, 2], 
                        c='red', marker='s', s=200, label='Key Waypoints', 
                        edgecolors='black', linewidth=2, alpha=0.9)
        
        # Add key waypoint labels
        for i, waypoint in enumerate(self.key_waypoints):
            final_ax.text(waypoint[0], waypoint[1], -waypoint[2] + 0.15, f'KWP{i}', 
                         fontsize=12, fontweight='bold', color='red', ha='center')
        
        # Plot intermediate waypoints (smaller, orange circles)
        intermediate_waypoints = [wp for wp in self.waypoints if wp not in self.key_waypoints]
        if intermediate_waypoints:
            intermediate_array = np.array(intermediate_waypoints)
            final_ax.scatter(intermediate_array[:, 0], intermediate_array[:, 1], -intermediate_array[:, 2], 
                           c='orange', marker='o', s=80, label='Intermediate Waypoints', alpha=0.7)
        
        # Plot planned path (green dashed line)
        planned_path = self.get_planned_path()
        planned_array = np.array(planned_path)
        final_ax.plot(planned_array[:, 0], planned_array[:, 1], -planned_array[:, 2], 
                     'g--', linewidth=3, alpha=0.8, label='Planned Path')
        
        # Plot actual flight path (blue solid line)
        if len(self.position_history) > 1:
            history_array = np.array(self.position_history)
            final_ax.plot(history_array[:, 0], history_array[:, 1], history_array[:, 2], 
                         'b-', linewidth=4, label='Actual Flight Path', alpha=0.9)
        
        # Plot takeoff position
        final_ax.scatter([0], [0], [0], c='green', marker='^', s=250, label='Takeoff', 
                        edgecolors='black', linewidth=2)
        final_ax.text(0, 0, 0.2, 'START', fontsize=12, fontweight='bold', 
                     color='green', ha='center')
        
        # Plot landing position
        if hasattr(self.vehicle_local_position, 'x') and len(self.position_history) > 0:
            final_pos = self.position_history[-1]
            final_ax.scatter([final_pos[0]], [final_pos[1]], [0], 
                           c='purple', marker='v', s=250, label='Landing', 
                           edgecolors='black', linewidth=2)
            final_ax.text(final_pos[0], final_pos[1], 0.2, 'END', fontsize=12, 
                         fontweight='bold', color='purple', ha='center')
        
        # Calculate path statistics
        if len(self.position_history) > 1:
            # Calculate total distance traveled
            total_distance = 0
            for i in range(1, len(self.position_history)):
                prev_pos = np.array(self.position_history[i-1])
                curr_pos = np.array(self.position_history[i])
                total_distance += np.linalg.norm(curr_pos - prev_pos)
            
            # Calculate planned distance
            planned_distance = 0
            for i in range(1, len(planned_path)):
                prev_pos = np.array(planned_path[i-1])
                prev_pos[2] = -prev_pos[2]  # Convert Z coordinate
                curr_pos = np.array(planned_path[i])
                curr_pos[2] = -curr_pos[2]  # Convert Z coordinate
                planned_distance += np.linalg.norm(curr_pos - prev_pos)
            
            # Add statistics to plot
            stats_text = f'Mission Statistics:\n'
            stats_text += f'Key Waypoints: {len(self.key_waypoints)}\n'
            stats_text += f'Total Waypoints: {len(self.waypoints)}\n'
            stats_text += f'Planned Distance: {planned_distance:.2f}m\n'
            stats_text += f'Actual Distance: {total_distance:.2f}m\n'
            stats_text += f'Flight Time: {time.time() - self.start_time:.1f}s'
            
            final_ax.text2D(0.02, 0.02, stats_text, transform=final_ax.transAxes, 
                           fontsize=10, verticalalignment='bottom',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Setup plot appearance
        final_ax.set_xlabel('X (North) [m]', fontsize=12)
        final_ax.set_ylabel('Y (East) [m]', fontsize=12)
        final_ax.set_zlabel('Z (Up) [m]', fontsize=12)
        final_ax.set_title('Complete Drone Flight Analysis\nKey Waypoints, Planned Path vs Actual Flight Path', 
                          fontsize=14, fontweight='bold')
        final_ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=11)
        final_ax.grid(True, alpha=0.3)
        
        # Set equal aspect ratio and adjust view
        if len(self.position_history) > 0:
            all_positions = np.array(self.position_history + planned_path)
            max_range = max(
                np.max(np.abs(all_positions[:, 0])),
                np.max(np.abs(all_positions[:, 1])),
                np.max(np.abs(all_positions[:, 2]))
            )
            final_ax.set_xlim([-max_range*1.1, max_range*1.1])
            final_ax.set_ylim([-max_range*1.1, max_range*1.1])
            final_ax.set_zlim([0, max_range*1.1])
        
        # Set a good viewing angle
        final_ax.view_init(elev=20, azim=45)
        
        plt.tight_layout()
        
        # Save the plot
        try:
            plt.savefig('drone_flight_path.png', dpi=300, bbox_inches='tight', 
                       facecolor='white', edgecolor='none')
            print("\n" + "="*50)
            print("MISSION COMPLETE!")
            print("="*50)
            print(f"Final flight analysis saved as 'drone_flight_path.png'")
            print("Plot includes:")
            print("- Red squares: Key waypoints (your original waypoints)")
            print("- Orange circles: Intermediate waypoints (auto-generated)")
            print("- Green dashed line: Planned flight path") 
            print("- Blue solid line: Actual flight path")
            print("- Green triangle: Takeoff position")
            print("- Purple triangle: Landing position")
            print("="*50)
        except Exception as e:
            print(f"Error saving plot: {e}")
        
        plt.show()
        plt.ion()  # Turn interactive mode back on


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

    print('Starting offboard control node with smooth trajectory following...')
    print('Features enabled:')
    print('- Dense waypoint generation with cubic spline interpolation')
    print('- Smooth trajectory following (no stopping at waypoints)')
    print('- Lookahead control for better path tracking')
    print('- Periodic status updates every 2 seconds')
    print('- Real-time 3D flight path visualization')
    print('- Comprehensive flight analysis saved as PNG')
    print('')
    print('Trajectory parameters:')
    print('- Default waypoint spacing: 0.3m')
    print('- Advance distance: 0.5m') 
    print('- Lookahead distance: 1.0m')
    print('- Install scipy for best spline interpolation: pip install scipy')
    print('')
    
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