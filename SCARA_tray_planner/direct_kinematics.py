import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math
import numpy as np


class ScaraDirectKinematics(Node):
    def __init__(self):
        super().__init__('direct_kinematics')
        
        # ---- Parameters (declare + get) ----
        self.declare_parameter('joint1', 'brazo1_joint')
        self.declare_parameter('joint2', 'brazo2_joint') 
        self.declare_parameter('joint3', 'prismatic_joint')
        self.declare_parameter('publish_rate', 50.0)      # Hz
    

        self.joint1 = self.get_parameter('joint1').get_parameter_value().string_value
        self.joint2 = self.get_parameter('joint2').get_parameter_value().string_value
        self.joint3 = self.get_parameter('joint3').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('publish_rate').value)
        
        # ---- State ----
        self.pj1 = 0.0
        self.vj1 = 0.0 
        self.pj2 = 0.0
        self.vj2 = 0.0
        self.pj3 = 0.0
        
        # ---- Transformations ----
        self.endEffector_x = 0.0
        self.endEffector_y = 0.0
        self.endEffector_z = 0.0
   
        self.dh_matrix = np.array([[0, 0, 0.195, 0], [0.155, 0, 0, 0], [0.13, math.pi, 0.044, 0], [0, 0, 0.080, 0]], dtype=float) # [a, alpha, Sj, thetaj]    

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'actual_pos', self._scara_configuration, qos)



    def _T_DH(self, a_ij, alpha_ij, s_i, theta_i):
        cos_theta = math.cos(theta_i)
        sin_theta = math.sin(theta_i)
        cos_alpha = math.cos(alpha_ij)
        sin_alpha = math.sin(alpha_ij)
        T_ij = np.array([
            [cos_theta, -sin_theta, 0, a_ij],
            [cos_alpha * sin_theta, cos_alpha * cos_theta, -sin_alpha, -s_i * sin_alpha],
            [sin_alpha * sin_theta, sin_alpha * cos_theta, cos_alpha, s_i * cos_alpha],
            [0, 0, 0, 1]
        ], dtype=float)
        return T_ij
    

    def _scara_forward_kinematics(self):
        # Update DH matrix with new joint values
        self.dh_matrix[0][3] = self.pj1
        self.dh_matrix[1][3] = self.pj2
        self.dh_matrix[3][2] = self.pj3

        DoF = self.dh_matrix.shape[0]
        T_ij = np.zeros((4, 4, DoF))
        for j in range(DoF):
            T_ij[:, :, j] = self._T_DH(self.dh_matrix[j, 0], self.dh_matrix[j, 1], self.dh_matrix[j, 2], self.dh_matrix[j, 3])

        # Cumulative transformation matrices
        T = np.zeros((4, 4, DoF))
        T[:, :, 0] = T_ij[:, :, 0]
        for i in range(1, DoF):
            T[:, :, i] = T[:, :, i-1] @ T_ij[:, :, i]

        # End-effector pose is T[:, :, -1]
        final_pos_pose = T[:, :, -1]
        
        self.endEffector_x = final_pos_pose[0, 3]
        self.endEffector_y = final_pos_pose[1, 3]
        self.endEffector_z = final_pos_pose[2, 3]

        self.get_logger().info(f'End Effector Position: x={self.endEffector_x:.4f} m, y={self.endEffector_y:.4f} m, z={self.endEffector_z:.4f} m')
    
            
    def _scara_configuration(self, msg: Twist):
        # The message sends angular velocity in deg/s. We convert it to rad/s
        # Angular position in deg, we convert it to rad
        # Linear position in mm, we convert it to m
        self.pj1 = msg.linear.x  # Joint 1 position (rad)
        self.pj2 = msg.linear.y  # Joint 2 position (rad)
        self.pj3 = msg.linear.z  # Joint 3 position (m)

        self._scara_forward_kinematics()

def main(args=None):
    rclpy.init(args=args)
    node = ScaraDirectKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()