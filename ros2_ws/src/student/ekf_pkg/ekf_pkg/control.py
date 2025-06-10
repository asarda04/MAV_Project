import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from interfaces.msg import Actuator
import sys
import math

sys.path.append('/workspaces/mavlab/')

from ros2_ws.src.student.tut_03.module_kinematics import quat_to_eul, ssa
from ros2_ws.src.student.tut_03.class_vessel import Vessel
from ros2_ws.src.student.tut_03.read_input import read_input


class control_node(Node):
    def __init__(self):
        super().__init__('control_node')

        self.wb = 1  # control bandwidth
        self.G = 0.1  # damping ratio
        self.wn = self.wb / math.sqrt(1 - 2 * self.G ** 2 + math.sqrt(4 * self.G ** 4 - 4 * self.G ** 2 + 2))


        vessel_params, hydrodynamic_data = read_input()
        vessel = Vessel(vessel_params, hydrodynamic_data)

        self.M = vessel.mass_matrix + vessel.added_mass_matrix()

        self.T = self.M[5, 5] / (-1 * vessel.N_r)
        self.K = -1 * vessel.N_delta / vessel.N_r

        self.m = self.T / self.K
        self.d = 1 / self.K

        self.Kp = 5  #self.wn ** 2 * self.T / self.K
        self.Kd = 0.25    #(2 * self.G * self.wn * self.T - 1) / self.K
        self.Ki = 0   #self.wn ** 3 * self.T / (10 * self.K)

        self.x = 0.0   # Current yaw
        self.xd = 0.0  # Desired yaw
        self.xp = 0.0  # Previous error
        self.xi = 0.0  # Integral term
        self.x_ = 0.0  # Current error
        self.yaw_rate = 0.0 
        self.dt_ready = True

        self.dt = 0.1
      #  self.time_sub = self.create_subscription(Float64MultiArray, '/ekf/timestep', self.time_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/sookshma_00/odometry', self.state_callback, 10)
        self.psi_sub = self.create_subscription(Float64, '/guidance/psi_d', self.psi_d_callback, 10)
        

        self.actuator_cmd_pub = self.create_publisher(Actuator, '/sookshma_00/actuator_cmd', 10)

    def psi_d_callback(self, data):
        self.xd = data.data
        x_ = ssa(self.x - self.xd, deg = True)

        if not self.dt_ready:
            self.get_logger().warn("dt not yet received. Skipping control computation.")
            return

        x_d = self.yaw_rate  #ssa(x_ - self.xp) / self.dt # derivative
        #self.xp = x_

        self.xi = ssa(self.xi + x_ * self.dt, deg = True)       # probably not necessary to multiply with dt
        max_int = np.deg2rad(60)                    # 60° maximum integral (Anti-windup clamping)
        self.xi = np.clip(self.xi, -max_int, max_int)

        delta = (self.Kp * x_ + self.Kd * x_d + self.Ki * self.xi)
        delta = np.clip(delta, -35, 35)  # ±35° rudder limit

        self.get_logger().info(f"Control Actuator rudder angle publish: {delta:.2f}°")

        # Publish proper actuator message
        cmd = Actuator()
        cmd.actuator_names = ["rudder"]
        cmd.actuator_values = [float(delta)]
        self.actuator_cmd_pub.publish(cmd)

    def state_callback(self, msg):
        q = [msg.pose.pose.orientation.w,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z]
        
        self.yaw_rate = np.rad2deg(msg.twist.twist.angular.z)

        print(self.yaw_rate)

        self.x =  np.rad2deg(quat_to_eul(q)[2]) # Extract yaw from quaternion

    def actuator_callback(self, data):
        self.x = np.array(data.actuator_values)

    def time_callback(self, data):
        self.dt = data.data[0] if data.data else 0.01
        self.dt_ready = True


def main(args=None):
    rclpy.init(args=args)
    node = control_node()
    rclpy.spin(node)
    rclpy.shutdown()
