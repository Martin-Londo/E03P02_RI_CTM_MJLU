#!/usr/bin/env python3

# Libraries ROS2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory

from glob import glob
import os
import pandas as pd
import numpy as np

# MACROS
pkg_path = get_package_share_directory('SCARA_tray_planner')
csv_install_path = os.path.join(pkg_path, "csv/dxf_waypoints_v5.csv")
csv_src_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "csv", "dxf_waypoints_v5.csv"
)

if os.path.exists(csv_install_path):
    path_to_csv = csv_install_path
elif os.path.exists(csv_src_path):
    path_to_csv = csv_src_path
else:
    raise FileNotFoundError("dxf_waypoints_v5.csv not found in either install or source locations.")

class Trajectory_planner_node(Node):

    def __init__(self):
        super().__init__("Trajectory_planner_node")

        self.get_logger().info("Waiting for DXF")

        # Info
        self.get_logger().info("Waiting for DXF")

        # Variables
        self.waypoints      = pd.read_csv(path_to_csv)
        self.waypoints['x'] = self.waypoints['x']/1000.0 * -1
        self.waypoints['y'] = self.waypoints['y']/1000.0 
        self.waypoints['z'] = self.waypoints['z']/1000.0
        self.figure_type    = self.waypoints['figure_type']
        self.figure_id      = self.waypoints['figure_id']

        # Info
        self.get_logger().info(f"# Waypoints: {len(self.waypoints)}")

        # Interpolation between waypoints
        self.interpolated_wp = self.interpolate(self.waypoints)

        # Save points to csv
        self.interpolated_wp.to_csv(os.path.join(pkg_path,"csv/interpolated_waypoints.csv"),index=False)

        # Info
        self.get_logger().info(f"# Interpolated Waypoints: {len(self.interpolated_wp)}")

        # Tray Publisher
        self.frame_id = "base_link" # Frame of reference for the path
        self.current_index = 0
        qos = QoSProfile(depth=10)

        self.path_pub = self.create_publisher(Path, "planned_trajectory", 10)
        self.twist_pub = self.create_publisher(Twist, "trajectory", qos)

        # Timer callback
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz

    def timer_callback(self):
        self.publish_planned_trajectory()
        self.publish_trajectory()

    def publish_planned_trajectory(self):
        if self.current_index < len(self.interpolated_wp):
            row = self.interpolated_wp.iloc[self.current_index]
            path_msg = Path()
            path_msg.header.frame_id = self.frame_id
            path_msg.header.stamp = self.get_clock().now().to_msg()

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(row['x']-0.02)*10
            pose.pose.position.y = float(row['y']-0.02)*10
            pose.pose.position.z = float(row['z'])*10
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Published planned trajectory with 1 pose at index {self.current_index} in frame '{self.frame_id}'")
        else:
            self.get_logger().info("All waypoints have been published.")

    def publish_trajectory(self):
        if self.current_index < len(self.interpolated_wp):
            row = self.interpolated_wp.iloc[self.current_index]
            twist_msg = Twist()
            twist_msg.linear.x = float(row['x'])
            twist_msg.linear.y = float(row['y'])
            twist_msg.linear.z = float(row['z'])
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0

            self.twist_pub.publish(twist_msg)
            self.get_logger().info(f"Published trajectory with 1 pose at index {self.current_index} in frame '{self.frame_id}'")
            self.current_index += 1
        else:
            self.get_logger().info("All waypoints have been published.")

    # Interpolation
    def quintic_interpolation(self, q0, qf, T, N, v0=0.0, vf=0.0, a0=0.0, af=0.0):
        t = np.linspace(0, T, N)
        a0_c = q0
        a1_c = v0
        a2_c = a0 / 2
        a3_c = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T**2) / (2 * T**3)
        a4_c = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T**2) / (2 * T**4)
        a5_c = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T**2) / (2 * T**5)
        q = a0_c + a1_c * t + a2_c * t**2 + a3_c * t**3 + a4_c * t**4 + a5_c * t**5
        return q

    # Point Density for diffrent DXF entities
    def get_point_density(self, figure_type):
        if figure_type in ["LINE", "LWPOLYLINE", "POLYLINE"]:       # Lines and Polylines get more points because only 2 WP
            return 80   
        elif figure_type in ["CIRCLE", "ARC"]:                      # Circles and Arcs get fewer points because they have many WP
            return 5     
        elif figure_type == "SPLINE":                               # Splines get a medium number of points    
            return 15    
        else:
            return 20

    def interpolate(self, waypoints):
        points = pd.DataFrame(columns=['x', 'y', 'z', 'figure_type', 'figure_id'])

        # Check if waypoints is empty
        if waypoints.empty:
            self.get_logger().warn("No waypoints to interpolate.")
            return points

        # Variables
        F_start_point = waypoints.iloc[0]

        for i in range(len(waypoints) - 1):
            # Current and next point
            actual_point = waypoints.iloc[i]
            next_point = waypoints.iloc[i + 1]

            # Extract data
            actual_point_id = actual_point['figure_id']
            next_point_id = next_point['figure_id']

            actual_point_type = actual_point['figure_type']


            if actual_point_id == next_point_id:                # Still in the same figure
                N = self.get_point_density(actual_point_type)   # Number of points based on figure type 
                T = 5.0                                         # DT parameter for interpolation

                ## Lower z axis to draw the figure
                points = pd.concat([points, pd.DataFrame({'x': [F_start_point['x']], 'y': [F_start_point['y']], 'z': 0, 'figure_type': [actual_point_type], 'figure_id': [actual_point_id]})], ignore_index=True)
                self.get_logger().info("Lower")

                # Interpolate between actual_point and next_point
                interpolated_x = self.quintic_interpolation(actual_point['x'], next_point['x'], T, N)
                interpolated_y = self.quintic_interpolation(actual_point['y'], next_point['y'], T, N)
                interpolated_z = self.quintic_interpolation(actual_point['z'], next_point['z'], T, N)

                # Add interpolated points to DataFrame
                points = pd.concat([points, pd.DataFrame({'x': interpolated_x, 'y': interpolated_y, 'z': interpolated_z, 'figure_type': actual_point_type, 'figure_id': actual_point_id})], ignore_index=True)

                self.get_logger().info("same_figure")

            else:
                self.get_logger().info("figure_change")

                N = self.get_point_density(actual_point_type)   # Number of points based on figure type 
                T = 5.0                                         # DT parameter for interpolation

                ## Connect last point to start point of figure
                # Interpolate between actual_point and F_start_point
                interpolated_x = self.quintic_interpolation(actual_point['x'], F_start_point['x'], T, N)
                interpolated_y = self.quintic_interpolation(actual_point['y'], F_start_point['y'], T, N)
                interpolated_z = self.quintic_interpolation(actual_point['z'], F_start_point['z'], T, N)
                points = pd.concat([points, pd.DataFrame({'x': interpolated_x, 'y': interpolated_y, 'z': interpolated_z, 'figure_type': actual_point_type, 'figure_id': actual_point_id})], ignore_index=True)

                ## Lift z axis to move over to next figure
                lift_height = 0.05
                points = pd.concat([points, pd.DataFrame({'x': [F_start_point['x']], 'y': [F_start_point['y']], 'z': [F_start_point['z'] + lift_height], 'figure_type': [actual_point_type], 'figure_id': [actual_point_id]})], ignore_index=True)
                self.get_logger().info("lift")

                ## Move to start point of next figure
                points = pd.concat([points, pd.DataFrame({'x': [next_point['x']], 'y': [next_point['y']], 'z': [next_point['z'] + lift_height], 'figure_type': [next_point['figure_type']], 'figure_id': [next_point['figure_id']]})], ignore_index=True)
                self.get_logger().info("Move to next figure")

                ## Update F_start_point for the new figure
                F_start_point = next_point

        return points

# Publish trajectory
def main(args=None):
    rclpy.init(args=args)
    node = Trajectory_planner_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()