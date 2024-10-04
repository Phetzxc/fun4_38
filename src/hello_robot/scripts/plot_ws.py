#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  
from spatialmath import *
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import random  

class WorkspacePlotter(Node):
    def __init__(self):
        super().__init__('workspace_plotter')
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(a=0, alpha=0, d=0.2, offset=0),
                rtb.RevoluteMDH(a=0, alpha=-pi/2, d=0, offset=-pi/2),
                rtb.RevoluteMDH(a=0.25, alpha=0, d=-0.02, offset=pi/2),
            ],
            tool=SE3.Rx(pi/2)@SE3.Tz(0.28),
            name="RRR_Robot"
        )
        self.calculate_ws()

    def calculate_ws(self):
        self.positions = []

        q1_range = np.linspace(3.14, -3.14, 50)
        q2_range = np.linspace(3.14, -3.14, 50)
        q3_range = np.linspace(3.14, -3.14, 50)

        for q1 in q1_range:
            for q2 in q2_range:
                for q3 in q3_range:
                    T_end_effector = self.robot.fkine([q1,q2,q3])
                    x, y, z =  T_end_effector.t
                    self.positions.append((x, y, z))  

        self.plot_workspace(self.positions)


    def plot_workspace(self, positions):
        """Plot the end-effector positions in 3D."""
        x_vals, y_vals, z_vals = zip(*positions) 

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')  
        ax.scatter(x_vals, y_vals, z_vals, c='b', marker='o', s=1)

        ax.set_title("End-Effector Workspace in 3D")
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")

        ax.set_box_aspect([1, 1, 1])  
        plt.show()  

def main(args=None):
    rclpy.init(args=args)
    node = WorkspacePlotter()
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
