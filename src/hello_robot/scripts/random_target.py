#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import PoseStamped
from spatialmath import *
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import random 

class RandomTargetNode(Node):
    def __init__(self):
        super().__init__('random_target_node')
        self.target_pub = self.create_publisher(PoseStamped, "/rand", 10)
        
        self.q1_range = np.linspace(3.14, -3.14, 50)
        self.q2_range = np.linspace(3.14, -3.14, 50)
        self.q3_range = np.linspace(3.14, -3.14, 50)

        self.create_timer(1.0, self.publish_random_target)

    def random_pose_in_ws(self):
        """Calculate end-effector position using MDH parameters."""
        q1 = np.random.choice(self.q1_range)
        q2 = np.random.choice(self.q2_range)
        q3 = np.random.choice(self.q3_range)
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(a=0, alpha=0, d=0.2, offset=0),          
                rtb.RevoluteMDH(a=0, alpha=-pi/2, d=0, offset=-pi/2),     
                rtb.RevoluteMDH(a=0.25, alpha=0, d=-0.02, offset=pi/2),   
            ],
            tool=SE3.Rx(pi/2) @ SE3.Tz(0.28),
            name="RRR_Robot"
        )
        T0e = self.robot.fkine([q1,q2,q3])
  
        x = T0e.t[0]
        y = T0e.t[1]
        z = T0e.t[2]
        return x, y, z 



    
    def publish_random_target(self):

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = "link_0"  
        x,y,z = self.random_pose_in_ws()
        msg.pose.position.x = x  
        msg.pose.position.y = y 
        msg.pose.position.z = z  

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0  

        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
