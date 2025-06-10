import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import sys
import math

sys.path.append('/workspaces/mavlab/')
from ros2_ws.src.student.tut_03.module_kinematics import quat_to_eul, ssa

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_node')

        # Parameters
        self.declare_parameter('ki', 0.002)
        self.declare_parameter('look_ahead_dist', 1.0)
        self.declare_parameter('goal_radius', 1.0)

        self.ki = self.get_parameter('ki').value
        self.look_ahead_dist = self.get_parameter('look_ahead_dist').value
        self.declare_parameter('kp', 1/self.look_ahead_dist)
        self.kp = self.get_parameter('kp').value
        self.goal_radius = self.get_parameter('goal_radius').value

        self.dt = 0.1 # Timer interval in seconds

        # State variables
        self.position = np.zeros(2)
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.cross_track_error = 0.0
        self.cross_track_error_integral = 0.0

        # Waypoints (list of [x, y] coordinates)
        self.waypoints = [
            [9.7, 27.8],
            [6.0, 6.0],
            [15.0, 6.0],
            [15.0, 20.0],
            [6.0, 20.0],
            [6.0,6.0]
        ]
        self.current_waypoint_idx = 1

        # ROS 2 subscriptions
        self.create_subscription(Odometry, '/sookshma_00/odometry', self.state_callback, 10)

        # ROS 2 publishers
        self.desired_heading_pub = self.create_publisher(Float64, '/guidance/psi_d', 10)  # Updated topic
        #self.current_heading_pub = self.create_publisher(Float64, '/guidance/current_heading', 10)
        #self.yaw_rate_pub = self.create_publisher(Float64, '/guidance/yaw_rate', 10)

        # Timer to trigger the guidance loop periodically
        self.create_timer(self.dt, self.guidance_loop)

        self.get_logger().info('ILOS Guidance Node started.')

    def state_callback(self, msg):
        # Extract position and orientation from Odometry message
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.yaw_rate = msg.twist.twist.angular.z

        quat = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ]
        # Convert quaternion to Euler angles
        self.yaw = quat_to_eul(quat)[2]

    def guidance_loop(self):
        # Wait for initial position data
        if np.all(self.position == 0):
            self.get_logger().info('Waiting for position data...')
            return

        prev_wp = self.waypoints[self.current_waypoint_idx - 1]
        curr_wp = self.waypoints[self.current_waypoint_idx]

        # Calculate path vector and position error
        path_vector = np.array(curr_wp) - np.array(prev_wp)                 
        path_unit = path_vector / (np.linalg.norm(path_vector) + 1e-8)
        pos_error = np.array(curr_wp) - self.position
        distance_to_wp = np.linalg.norm(pos_error)

        # Compute cross-track error
        cross_track_error = np.cross(pos_error, path_unit)
      # self.cross_track_error_integral += (cross_track_error*self.look_ahead_dist/(self.look_ahead_dist**2 + (cross_track_error + (self.ki/self.kp)*self.cross_track_error_integral)**2)) * self.dt

      # self.cross_track_error_integral += (cross_track_error / (self.look_ahead_dist**2 + cross_track_error**2)) * self.dt
        self.cross_track_error_integral += cross_track_error * self.dt



        # Compute desired heading based on the ILOS method
        path_heading = np.arctan2(path_vector[1], path_vector[0])
        los_correction = -np.arctan(self.kp * cross_track_error + self.ki * self.cross_track_error_integral)
        desired_heading = ssa(path_heading + los_correction)

        # Log useful data
        self.get_logger().info(f'Distance to WP: {distance_to_wp:.2f}, CT Error: {cross_track_error:.2f}')
        self.get_logger().info(f'Yaw: {np.degrees(self.yaw):.2f}°, Desired: {np.degrees(desired_heading):.2f}°')

        # Publish desired heading
        self.publish_angle(self.desired_heading_pub, desired_heading)
        #self.publish_angle(self.current_heading_pub, self.yaw)
        #self.publish_angle(self.yaw_rate_pub, self.yaw_rate)

        # Check for waypoint arrival
        if distance_to_wp < self.goal_radius:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}')
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.current_waypoint_idx = 0
                self.get_logger().info('Restarting waypoint sequence.')
            self.cross_track_error_integral = 0.0

    def publish_angle(self, pub, angle_rad):
        msg = Float64()
        msg.data = np.degrees(angle_rad)  # Convert radians to degrees
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
