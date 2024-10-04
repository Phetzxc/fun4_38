#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopTwistKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard_node')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = 0.5  
        self.turn = 1.0   

        self.move_bindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'j': (0, 0, 0, 1),
            'l': (0, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'k': (0, 0, 0, 0),
            't': (0, 0, 1, 0),
            'b': (0, 0, -1, 0),
            'a': (1, 0, 0, 0),
            's': (0, 1, 0, 0),
            'd': (0, 0, 1, 0),
            'f': (-1, 0, 0, 0),
            'g': (0, -1, 0, 0),
            'h': (0, 0, -1, 0),
        }

        self.speed_bindings = {
            'z': (0.9, 0.9),  
            'w': (1.1, 1.0),  
            'x': (0.9, 1.0),  
            'e': (1.0, 1.1),  
            'c': (1.0, 0.9),  
        }

        self.get_logger().info("""
        Teleop Twist Keyboard Node Initialized.
        Use the following keys to control the robot:

        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        Speed control:
           z : decrease both linear and angular speed
           w : increase linear speed
           x : decrease linear speed
           e : increase angular speed
           c : decrease angular speed

        Special Moves (one-time):
           a : Move 0.5 1 time and stop in x direction
           s : Move 0.5 1 time and stop in y direction
           d : Move 0.5 1 time and stop in z direction
           f : Move -0.5 1 time and stop in x direction
           g : Move -0.5 1 time and stop in y direction
           h : Move -0.5 1 time and stop in z direction

        Press 'q' to quit.
        """)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()

                if key == 'q':
                    self.get_logger().info("Exiting... 'q' was pressed.")
                    raise KeyboardInterrupt

                if key in self.move_bindings.keys():
                    x, y, z, th = self.move_bindings[key]

                    twist = Twist()
                    twist.linear.x = x * 0.5 if key in ['a', 'f'] else 0.0
                    twist.linear.y = y * 0.5 if key in ['s', 'g'] else 0.0
                    twist.linear.z = z * 0.5 if key in ['d', 'h'] else 0.0
                    twist.angular.z = th * self.turn

                    self.publisher_.publish(twist)
                    self.get_logger().info(f"Published one-time move: {twist}")

                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    self.get_logger().info("Published reset to stop.")
                
                elif key in self.speed_bindings.keys():
                    self.speed *= self.speed_bindings[key][0]
                    self.turn *= self.speed_bindings[key][1]
                    self.get_logger().info(f"Speed: {self.speed}, Turn: {self.turn}")
                    continue

        except Exception as e:
            self.get_logger().error(f"Error in teleop_twist_keyboard: {e}")
        finally:
            twist = Twist()
            self.publisher_.publish(twist)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TeleopTwistKeyboardNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down teleop_twist_keyboard due to 'q' press or Ctrl+C.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
