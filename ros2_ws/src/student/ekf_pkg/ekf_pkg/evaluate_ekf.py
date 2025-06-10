import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import math

import sys
sys.path.append('/workspaces/mavlab/')

from ros2_ws.src.student.tut_03.module_kinematics import quat_to_eul,eul_to_quat

class EKFEvaluator(Node):
    def __init__(self):
        super().__init__('ekf_evaluator')

        self.x_ekf, self.y_ekf, self.yaw_ekf = [], [], []
        self.x_uwb, self.y_uwb = [], []
        self.errors, self.rmse_vals = [], []

        self.innovations = []
        self.covariances = []
        self.h_vals = []
        self.full_states = []

        self.create_subscription(Odometry, '/ekf/odom', self.ekf_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/sookshma_00/uwb', self.uwb_callback, 10)
        self.create_subscription(Float64MultiArray, '/ekf/innovation', self.innovation_callback, 10)
        self.create_subscription(Float64MultiArray, '/ekf/cov_diag', self.cov_callback, 10)
        self.create_subscription(Float64MultiArray, '/ekf/timestep', self.h_callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(6, 3, figsize=(16, 18))

    def quaternion_to_yaw(self, q):
        [_,_,yaw] = quat_to_eul([q.x,q.y,q.z,q.w])
        return yaw

    def ekf_callback(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        vx, vy, vz = msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z
        wx, wy, wz = msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z

        roll, pitch, _ = quat_to_eul([msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z,
                                      msg.pose.pose.orientation.w])

        self.x_ekf.append(x)
        self.y_ekf.append(y)
        self.yaw_ekf.append(yaw)
        self.full_states.append([vx, vy, vz, wx, wy, wz, x, y, msg.pose.pose.position.z, roll, pitch, yaw])

        if self.x_uwb:
            error = math.sqrt((x - self.x_uwb[-1])**2 + (y - self.y_uwb[-1])**2)
            self.errors.append(error)
            self.rmse_vals.append(np.sqrt(np.mean(np.array(self.errors)**2)))

        self.plot()

    def uwb_callback(self, msg: PoseWithCovarianceStamped):
        self.x_uwb.append(msg.pose.pose.position.x)
        self.y_uwb.append(msg.pose.pose.position.y)

    def innovation_callback(self, msg: Float64MultiArray):
        self.innovations.append(msg.data)

    def cov_callback(self, msg: Float64MultiArray):
        self.covariances.append(msg.data)

    def h_callback(self, msg: Float64MultiArray):
        self.h_vals.append(msg.data[0])

    def plot(self):
        for i in range(6):
            for j in range(3):
                self.ax[i][j].clear()

        # XY Trajectory
        self.ax[0][0].plot(self.x_ekf, self.y_ekf, label='EKF', color='blue')
        self.ax[0][0].plot(self.x_uwb, self.y_uwb, label='UWB', color='orange', linestyle='dotted')
        self.ax[0][0].set_title('XY Trajectory')
        self.ax[0][0].legend()
        self.ax[0][0].grid()

        # Yaw
        self.ax[0][1].plot(np.rad2deg(self.yaw_ekf), label="Yaw (deg)")
        self.ax[0][1].set_title('Yaw')
        self.ax[0][1].legend()
        self.ax[0][1].grid()

        # Position Error
        self.ax[0][2].plot(self.errors, label='Position Error')
        self.ax[0][2].set_title('EKF vs UWB Error')
        self.ax[0][2].legend()
        self.ax[0][2].grid()

        # RMSE
        if self.rmse_vals:
            self.ax[1][0].plot(self.rmse_vals, label='Position RMSE')
            self.ax[1][0].set_title('Cumulative RMSE')
            self.ax[1][0].legend()
            self.ax[1][0].grid()

        # Innovation
        if self.innovations:
            data = np.array(self.innovations)
            for i in range(data.shape[1]):
                self.ax[1][1].plot(data[:, i], label=f"ỹ[{i}]")
            self.ax[1][1].set_title("Innovation (y - Cx)")
            self.ax[1][1].legend()
            self.ax[1][1].grid()

        # Covariance Diag
        if self.covariances:
            data = np.array(self.covariances)
            for i in range(data.shape[1]):
                self.ax[1][2].plot(data[:, i], label=f"P[{i},{i}]")
            self.ax[1][2].set_title("Covariance Diagonal")
            self.ax[1][2].legend()
            self.ax[1][2].grid()

        # Plot all estimated states (full state vector)
        if self.full_states:
            full_data = np.array(self.full_states)
            labels = ['u', 'v', 'w', 'p', 'q', 'r', 'x', 'y', 'z', 'phi', 'theta', 'psi']
            for i in range(12):
                row, col = divmod(i, 3)
                self.ax[row+2][col].plot(full_data[:, i], label=labels[i])
                self.ax[row+2][col].set_title(f'Est. State: {labels[i]}')
                self.ax[row+2][col].legend()
                self.ax[row+2][col].grid()

        plt.tight_layout()
        plt.savefig("/tmp/ekf_eval_debug.png")


def main(args=None):
    rclpy.init(args=args)
    node = EKFEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
