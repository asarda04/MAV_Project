import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from collections import deque
from interfaces.msg import Actuator
from std_msgs.msg import Float64MultiArray
import sys
sys.path.append('/workspaces/mavlab/')

# Custom imports
from ros2_ws.src.student.tut_03.module_kinematics import quat_to_eul, eul_to_quat
from ros2_ws.src.student.tut_03.read_input import read_input
from ros2_ws.src.student.tut_03.class_vessel import Vessel

class EKF_Node(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        # State vector: [u, v, w, p, q, r, x, y, z, phi, theta, psi]
        self.x_k = np.zeros(12)
        self.x_k[6:9] = [14,22,0]
        self.P_k = np.diag([0.1, 0.1, 0.01, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
        
        # Time management
        self.last_prediction = self.get_clock().now()
        self.delta_buffer = deque(maxlen=10)
        self._jacobian_eps = 1e-5 * np.ones(12)
        
        # Sensor buffers
        self.imu_data = None
        self.uwb_data = None
        
        # Process noise (tuned)
        self.Q = np.diag([0.1, 0.1, 0.01, 0.5, 0.5, 0.5, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        self.R = np.eye(9) * 0.1
        
        # Measurement matrix
        self.C = np.zeros((9, 12))
        self.C[0:3, 9:12] = self.C[3:6, 3:6] = self.C[6:9, 6:9] = np.eye(3)
        
        # Vessel model
        vessel_params, hydrodynamic_data = read_input()
        self.vessel = Vessel(vessel_params, hydrodynamic_data)
        
        # ROS setup
        self.create_timer(0.01, self.predict)  # 100Hz prediction
        self.create_timer(0.1, self.update)    # 10Hz update
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, "/sookshma_00/imu/data", self.imu_callback, 10)
        self.uwb_sub = self.create_subscription(PoseWithCovarianceStamped, "/sookshma_00/uwb", self.uwb_callback, 10)
        self.act_sub = self.create_subscription(Actuator, '/sookshma_00/actuator_cmd', self.actuator_callback, 10)
        
        # Publishers
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        self.innov_pub = self.create_publisher(Float64MultiArray, '/ekf/innovation', 10)
        self.pub_h = self.create_publisher(Float64MultiArray, '/ekf/timestep', 10)

    def actuator_callback(self, msg):
        if "rudder" in msg.actuator_names:
            idx = msg.actuator_names.index("rudder")
            self.delta_buffer.append(
                (self.get_clock().now().nanoseconds, msg.actuator_values[idx])
            )

    def imu_callback(self, msg):
        # Initialize orientation from first measurement
        if self.imu_data is None:
            self.x_k[9:12] = quat_to_eul([
                 msg.orientation.w, msg.orientation.x,
                 msg.orientation.y, msg.orientation.z
            ])
            
        self.imu_data = {
            'orientation': quat_to_eul([
                msg.orientation.w, msg.orientation.x, 
                msg.orientation.y, msg.orientation.z
            ]),
            'angular_vel': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'covariance': {
                'orient': np.reshape(msg.orientation_covariance, (3, 3)),
                'angular': np.reshape(msg.angular_velocity_covariance, (3, 3))
            }
        }

    def uwb_callback(self, msg):
        self.uwb_data = {
            'position': [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ],
            'covariance': np.reshape(msg.pose.covariance, (6, 6))[0:3, 0:3]
        }

    def get_delta(self):
        now = self.get_clock().now().nanoseconds
        if not self.delta_buffer:
            return 0.0
        
        # Find nearest timestamp
        closest = min(self.delta_buffer, key=lambda x: abs(x[0] - now))
        return closest[1]

    def vessel_jacobian(self, x, u):
        eps = 1e-5
        n = len(x) + 1 
        J = np.zeros((n, n))
        #f0 = self.vessel.vessel_ode(0, x)
        x_aug = np.append(x,u)
        
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = eps
            J[:, i] = (self.vessel.vessel_ode(0, x_aug + dx) - self.vessel.vessel_ode(0, x_aug - dx)) / (2*eps)
        
        return J[:-1,:-1]  # Exclude control input dimension

    def predict(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_prediction).nanoseconds * 1e-9
        self.last_prediction = current_time
        
        delta = self.get_delta()
        x_aug = np.append(self.x_k, delta)
        
        # State prediction
        x_dot = self.vessel.vessel_ode(0, x_aug)[:-1]
        self.x_k += dt * x_dot
        
        # Analytical Jacobian (implement based on vessel model)
        A = self.vessel_jacobian(self.x_k, delta)
        F = np.eye(12) + dt * A
        
        # Covariance prediction
        self.P_k = F @ self.P_k @ F.T + self.Q
        self.P_k = np.clip(self.P_k, -1e3, 1e3)  # Stability
        self.pub_h.publish(Float64MultiArray(data=[dt]))

    def update(self):
        if self.imu_data is None or self.uwb_data is None:
            return

        # Measurement vector
        y = np.concatenate([
            self.imu_data['orientation'],
            self.imu_data['angular_vel'],
            self.uwb_data['position']
        ])
        
        # Innovation calculation
        y_hat = self.C @ self.x_k
        innovation = y - y_hat
        
        # Measurement noise
        R = np.block([
            [self.imu_data['covariance']['orient'], np.zeros((3,3)), np.zeros((3,3))],
            [np.zeros((3,3)), self.imu_data['covariance']['angular'], np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), self.uwb_data['covariance']]
        ]) + 1e-6*np.eye(9)
        
        # Kalman gain
        S = self.C @ self.P_k @ self.C.T + R
        K = self.P_k @ self.C.T @ np.linalg.pinv(S)
        
        # State update
        self.x_k += K @ innovation
        self.P_k = (np.eye(12) - K @ self.C) @ self.P_k

        #print("Estimated innovation: ", self.C@self.x_k - y)
        print("Estimated State:", self.x_k)
        print("Measurement :", y)
        
        # Publish results
        self.publish_odometry()
        self.innov_pub.publish(Float64MultiArray(data=innovation.tolist()))

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Position
        msg.pose.pose.position.x = self.x_k[6]
        msg.pose.pose.position.y = self.x_k[7]
        msg.pose.pose.position.z = self.x_k[8]
        
        # Orientation
        q = eul_to_quat(self.x_k[9:12])
        msg.pose.pose.orientation.x = q[1]
        msg.pose.pose.orientation.y = q[2]
        msg.pose.pose.orientation.z = q[3]
        msg.pose.pose.orientation.w = q[0]
        
        # Velocity
        msg.twist.twist.linear.x = self.x_k[0]
        msg.twist.twist.linear.y = self.x_k[1]
        msg.twist.twist.linear.z = self.x_k[2]
        
        self.ekf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKF_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
