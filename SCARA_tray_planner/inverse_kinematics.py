import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import numpy as np

class ScaraInverseKinematics(Node):
    def __init__(self):
        super().__init__('scara_inverse_kinematics')
        
        # Variables
        self.r1             = 0.155    # Length of first arm [m]
        self.r2             = 0.13     # Length of second arm [m]
        self.min_prismatic  = -0.242   # Minimum extension of prismatic joint [m]
        self.max_prismatic  = 0.242    # Max prismatic extension [m]

        # Desired position
        self.px             = 0.285  # [m]
        self.py             = 0.0    # [m]
        self.pz             = 0.241  # [m]
        
        # Output joint configuration
        self.th1            = 0.0  # [rad]
        self.th2            = 0.0  # [rad]
        self.s3             = 0.0  # [m]
        self.flag           = False# Flag to indicate new position received
        self.zeros          = [0.0, 0.0, 0.0] # Default return value

        # Info
        self.get_logger().info(f'SCARA Inverse Kinematics node started')
        self.get_logger().info(f'Robot parameters: L1: {self.r1}m, L2: {self.r2}m, Prismatic range: {self.max_prismatic-self.min_prismatic}m')

        # Callback group for handling callbacks in parallel
        #self.cb = ReentrantCallbackGroup()

        # Publisher
        self.joinStates_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.jsmsg = JointState()
        self.jsmsg.header.frame_id = "base_link"
        self.jsmsg.name = ['baseprincipal_brazo1_joint', 'brazo2_joint', 'prismatic_joint']
        
        self.twist_pub = self.create_publisher(Twist, 'actual_pos', 10)
        self.tmsg = Twist()

        # Subscriber
        qos = QoSProfile(depth=10)
        self.tray_sub = self.create_subscription(Twist, 'trajectory', self._scara_configuration, qos)#, callback_group=self.cb)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)#, callback_group=self.cb)

    def timer_callback(self):
        # Info
        self.get_logger().info("In Timer Callback")
        self.publish_inverse_kinematics()
        self.publish_inverse_kinematics_twist()

    def publish_inverse_kinematics(self):
        # header
        now = self.get_clock().now()
        
        self.jsmsg.header.stamp = now.to_msg()

        self.solution = self._scara_inverse_kinematics()

        self.th1 = self.pj1
        self.th2 = self.pj2
        self.s3  = self.pj3

        # position
        self.jsmsg.position = [self.th1, self.th2, self.s3]
        
        # velocity
        self.jsmsg.velocity = [0.0, 0.0, 0.0]
        self.jsmsg.effort = [0.0, 0.0,0.0]
        
        # publish the message
        self.joinStates_pub.publish(self.jsmsg)

    def publish_inverse_kinematics_twist(self):
        # Create and publish Twist message
        self.tmsg.linear.x = (self.pj1)  # Convert to degrees
        self.tmsg.linear.y = (self.pj2)  # Convert to degrees
        self.tmsg.linear.z = (self.pj3)  # Keep z in m

        self.twist_pub.publish(self.tmsg)

        self.get_logger().debug(f'Published joint angles: th1={math.degrees(self.th1):.2f}°, th2={math.degrees(self.th2):.2f}°, z={self.pz:.2f}m')

    def _scara_inverse_kinematics(self):
        
        D = round((self.px**2 + self.py**2 - self.r1**2 - self.r2**2) / (2 * self.r1 * self.r2),6)
        if abs(D) > 1.0:
            self.get_logger().warn(f'Position out of reach: D = {D}')
            return
        #Two options for theta2: elbow up and elbow down
        theta2_opt = [math.acos(D), -math.acos(D)]
        theta1_opt = []
 
        for theta2 in theta2_opt:
            th1 = None
            if abs(math.degrees(theta2)) > 360:
                self.get_logger().warn(f'Position out of reach: |theta2| > 120 deg ({math.degrees(theta2):.2f} deg)')
                theta1_opt.append(None)
                continue
 
            mat = np.array([[self.r1 + self.r2 * math.cos(theta2), -self.r2 * math.sin(theta2)],
                            [self.r2 * math.sin(theta2), self.r1 + self.r2 * math.cos(theta2)]])
            vec = np.array([self.px, self.py])
 
            if np.linalg.det(mat) == 0:
                self.get_logger().warn('Singular configuration')
                theta1_opt.append(None)
                continue
 
            mat_inv = np.linalg.inv(mat)
            sol = mat_inv @ vec
            th1 = math.atan2(sol[1], sol[0])
 
            if abs(math.degrees(th1)) > 360:
                self.get_logger().warn(f'Position out of reach: |theta1| > 90 deg ({math.degrees(th1):.2f} deg)')
                th1 = None
 
            theta1_opt.append(th1)
 
        # Select the first valid (th1, th2) pair
        found_valid = False
        prismatic = self.pz
        for idx, th1 in enumerate(theta1_opt):
            th2 = theta2_opt[idx]
            if th1 is not None and th2 is not None and abs(prismatic) <= 14.0:
                self.pj1 = th1
                self.pj2 = th2
                self.pj3 = prismatic  # Extension of prismatic joint (m)
                found_valid = True
                break
 
        if not found_valid:
            self.get_logger().warn('No valid solution for joint angles; keeping previous values.')
            
        return       

    def _scara_configuration(self, msg: Twist):

        self.px = (msg.linear.x - 0.2)    # [m]
        self.py = (msg.linear.y)          # [m]
        self.pz = (msg.linear.z)          # [m]

        self.get_logger().info(f'Received desired position: x={self.px:.4f}m, y={self.py:.4f}m, z={self.pz:.4f}m')
        self.flag = True

        # fire one publish immediately
        self.publish_inverse_kinematics()
        
    
def main():
    rclpy.init()
    node = ScaraInverseKinematics() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    