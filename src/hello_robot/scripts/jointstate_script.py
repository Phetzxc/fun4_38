#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi
from spatialmath import SE3
import numpy as np
import roboticstoolbox as rtb
from tf2_ros import TransformListener, Buffer, TransformException
from lab1_interfaces.srv import SetMode
from geometry_msgs.msg import TransformStamped, PoseStamped ,Twist
from std_msgs.msg import String

class ModeServiceNode(Node):
    def __init__(self):
        super().__init__('mode_service_node')
        self.ref_mode = 0  
        self.current_twist = Twist()  
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_sub = self.create_subscription(PoseStamped, "/rand", self.target_random_callback, 10)
        self.end_eff_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.create_timer(1.0, self.publish_end_eff)
        self.target2_rviz_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.create_timer(1.0, self.publish_rand_target2_rviz)
        self.timer = self.create_timer(1.0, self.lookup_transform_callback)
        self.current_position = None
        self.target_frame = 'end_effector'
        self.source_frame = 'link_0'
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.name = ["joint_1", "joint_2", "joint_3"]

        self.Kp = 0 
        self.Ki = 0 
        self.Kd = 0 
        self.integral_error = np.zeros(3) 
        self.prev_error = np.zeros(3) 
        self.send_goal = True
        self.movement_start_time = 0  
        self.movement_duration = 7.0  
        self.singularity_pub = self.create_publisher(String, '/singularity_warning', 10)

        self.srv = self.create_service(SetMode, 'mode', self.set_mode_callback)
        self.mode = 0
        self.new_target = True
        self.movement_start_time = 0  
        self.movement_duration = 7.0  
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.teleop_callback, 10)

        self.ref = None
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(a=0, alpha=0, d=0.2, offset=0),       
                rtb.RevoluteMDH(a=0, alpha=-pi/2, d=0, offset=-pi/2),     
                rtb.RevoluteMDH(a=0.25, alpha=0, d=-0.02, offset=pi/2),  
            ],
            tool=SE3.Rx(pi/2) @ SE3.Tz(0.28),
            name="RRR_Robot"
        )

        self.q_sol = [0.0, 0.0, 0.0]
        self.q_now = [0.0, 0.0, 0.0] 
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.goal_position = np.array([0.0, -0.2, 0.73]) 
        self.q_before = [0,0,0]
    def publish_rand_target2_rviz(self):

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = "link_0"  
        if self.mode == 2:
            msg.pose.position.x = self.goal_position[0] 
            msg.pose.position.y = self.goal_position[1] 
            msg.pose.position.z = self.goal_position[2]     


        self.target2_rviz_pub.publish(msg)

    def detect_singularity(self, J):
        condition_number = np.linalg.cond(J)
        singularity_threshold = 1000 

        if condition_number > singularity_threshold:
            self.get_logger().warn("Near singularity detected!")
            warning_msg = String()
            warning_msg.data = "Singularity  Robot will stop."
            self.singularity_pub.publish(warning_msg)
            return True
        return False

    def teleop_callback(self, msg):
        self.current_twist = msg  
        v_linear = msg.linear  
        v_angular = msg.angular  

        if self.ref_mode == 1:  
            self.move_with_base(v_linear, v_angular)
        elif self.ref_mode == 0:
            self.move_with_end_effector(v_linear, v_angular)

    def move_with_end_effector(self, v_linear, v_angular):
        v = np.array([v_linear.x, v_linear.y, v_linear.z])  

        J = self.robot.jacob0(self.q_now)
        J_linear = J[0:3, :]  

        q_dot = np.linalg.pinv(J_linear) @ v  

        q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)

        q_new = self.q_now + q_dot_scaled * self.dt

        q_new = self.normalize_joint_angles(q_new)
        if self.check_joint_limits(q_new):
            self.q_now = q_new

        self.publish_joint_state()
        
    def publish_end_eff(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = "link_0"  
        msg.pose.position.x = self.robot.fkine(self.q_now).t[0]  
        msg.pose.position.y = self.robot.fkine(self.q_now).t[1]  
        msg.pose.position.z = self.robot.fkine(self.q_now).t[2]   

        self.end_eff_pub.publish(msg)
        
    def target_random_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        if self.mode == 2 and self.new_target:
            self.q_sol = [x, y, z]
            self.new_target = False
            self.goal_position = np.array([x, y, z])
            self.get_logger().info(f"Received target: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            self.movement_start_time = self.get_clock().now().seconds_nanoseconds()[0]  

        
    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.name

        msg.position = list(map(float, self.q_now)) 

        self.joint_pub.publish(msg)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def normalize_joint_angles(self, q):
        return [self.normalize_angle(angle) for angle in q]

    def check_joint_limits(self, q):
        for i, angle in enumerate(q):
            if not -pi <= angle <= pi:
                self.get_logger().warning(f"Joint {i+1} out of : {angle:.2f}.")
                return False
        return True

    def velocity_scaling(self, q, q_dot):
        scaled_q_dot = []
        for i, (angle, velocity) in enumerate(zip(q, q_dot)):
            distance_to_limit = min(abs(pi - angle), abs(-pi - angle))
            scaling_factor = max(0.1, distance_to_limit / pi)  
            scaled_q_dot.append(velocity * scaling_factor)
        return np.array(scaled_q_dot)

    def move_with_base(self, v_linear, v_angular):
        v = np.hstack((v_linear.x, v_linear.y, v_linear.z, v_angular.x, v_angular.y, v_angular.z))

        J = self.robot.jacob0(self.q_now)  
        q_dot = np.linalg.pinv(J) @ v  

        q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)

        q_new = self.q_now + q_dot_scaled * self.dt

        q_new = self.normalize_joint_angles(q_new)
        if self.check_joint_limits(q_new):
            self.q_now = q_new

        self.publish_joint_state()


    def sim_loop(self):
        if self.mode == 0 :
            self.Kp = 4 
            self.Ki = 0.01 
            self.Kd = 0.02 
            tolerance = 0.01  

            T_end_effector = self.robot.fkine(self.q_now)
            current_position = T_end_effector.t 

            error = self.goal_position - current_position

            
            if np.linalg.norm(error) < tolerance:
                self.send_goal = False
                # self.get_logger().info("End-effector has reached the target position.")
                return

            self.integral_error += error * self.dt  
            derivative_error = (error - self.prev_error) / self.dt 
            self.prev_error = error  

            v = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error

            J = self.robot.jacob0(self.q_now)  
            J_linear = J[0:3, :]  

            
            q_dot = np.linalg.pinv(J_linear) @ v 
            
            q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)

            
            q_new = self.q_now + q_dot_scaled * self.dt

           
            q_new = self.normalize_joint_angles(q_new)
            if self.check_joint_limits(q_new):
                self.q_now = q_new

   
            self.publish_joint_state()
            self.publish_end_eff()
        if self.mode == 1:
            self.Kp = 4  
            self.Ki = 0.01 
            self.Kd = 0.02 
            v_linear = self.current_twist.linear  
            v_angular = self.current_twist.angular 

            if self.ref_mode == 1:  
                J = self.robot.jacob0(self.q_now)  
                J_linear = J[0:3, :] 

                if self.detect_singularity(J_linear):
                    self.q_now = self.q_now  
                    self.publish_joint_state() 
                    return  

                v = np.array([v_linear.x, v_linear.y, v_linear.z])  
                q_dot = np.linalg.pinv(J_linear) @ v  
            elif self.ref_mode == 0:  
                J_e = self.robot.jacobe(self.q_now) 
                J_linear = J_e[0:3, :] 
                if self.detect_singularity(J_linear):
                    self.q_now = self.q_now  
                    self.publish_joint_state()      
                    return

                v = np.array([v_linear.x, v_linear.y, v_linear.z])  
                q_dot = np.linalg.pinv(J_linear) @ v 
            # elif self.ref_mode == 3:  
            #     J = self.robot.jacob0(self.q_now)
            #     v = np.hstack((v_linear.x, v_linear.y, v_linear.z, v_angular.x, v_angular.y, v_angular.z))

            #     if self.detect_singularity(J):
            #         self.q_now = self.q_now  
            #         self.publish_joint_state()  
            #         return

            #     q_dot = np.linalg.pinv(J) @ v  

            q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)

            q_new = self.q_now + q_dot_scaled * self.dt

            q_new = self.normalize_joint_angles(q_new)
            if self.check_joint_limits(q_new):
                self.q_now = q_new

            self.publish_joint_state()
            self.publish_end_eff()
        if self.mode == 2 and self.new_target == False:
            self.Kp = 5.5 
            self.Ki = 0.01 
            self.Kd = 0.03 
            tolerance = 0.01  

            T_end_effector = self.robot.fkine(self.q_now)
            self.current_position = T_end_effector.t  

            error = self.goal_position - self.current_position

            if np.linalg.norm(error) < tolerance:
                self.get_logger().info(f"End-effector has reached the target position.\n Position: {self.current_position}")
                self.get_logger().info(f"Time to move: {self.get_clock().now().seconds_nanoseconds()[0] - self.movement_start_time} seconds.")
                self.new_target = True 
                return

            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            elapsed_time = current_time - self.movement_start_time
            if elapsed_time > 7:
                self.get_logger().info(f"End-effector has reached the target position.\n Position: {self.current_position}")
                self.get_logger().warning(f"Time to move: {self.get_clock().now().seconds_nanoseconds()[0] - self.movement_start_time} seconds.")
                self.new_target = True  
                return
          
            self.integral_error += error * self.dt
            derivative_error = (error - self.prev_error) / self.dt
            self.prev_error = error 

            v = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
            J = self.robot.jacob0(self.q_now) 
            J_linear = J[0:3, :]  
            q_dot = np.linalg.pinv(J_linear) @ v  
            q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)
            q_new = self.q_now + q_dot_scaled * self.dt

            q_new = self.normalize_joint_angles(q_new)
            if self.check_joint_limits(q_new):
                self.q_now = q_new
            self.publish_joint_state()
            self.publish_end_eff()

            
    def lookup_transform_callback(self):
        try:
            now = rclpy.time.Time(seconds=0)
            transform = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, now)
            self.process_transform(transform)

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')

    def process_transform(self, transform: TransformStamped):
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        matrix = SE3(tx, ty, tz)

        self.q_sol, *_ = self.robot.ikine_LM(matrix, q0=self.q_now, tol=1e-6)

    def set_mode_callback(self, request, response):
        mode = request.mode.data
        if mode == 0:
            x = request.x.data
            y = request.y.data
            z = request.z.data
            self.get_logger().info(f'Received Mode 0: Move robot to x={x}, y={y}, z={z}')
            response.result.data = 1
            self.mode = 0
            matrix = SE3(x, y, z)
            ik_solution = self.robot.ikine_LM(matrix, mask=[1, 1, 1, 0, 0, 0])
            self.get_logger().info(f'Received {ik_solution.q}')
            # ik_solution = SE3(self.q_sol[0],self.q_sol[1],self.q_sol[2])
            pos = self.robot.fkine(ik_solution.q).t
            if abs(abs(pos[0]) - abs(x)) <0.01 and abs(abs(pos[1]) - abs(y)<0.01) and abs(abs(pos[2]) - abs(z)<0.01) :
            # if self.send_goal == True:
                self.goal_position = np.array([x, y, z])
                self.q_sol, *_ = self.robot.ikine_LM(matrix, q0=self.q_now, tol=1e-6)
                response.q0sol.data = self.q_sol[0]
                response.q1sol.data = self.q_sol[1]
                response.q2sol.data = self.q_sol[2]
                self.send_goal == True
                self.q_before = self.q_sol
                response.ipk.data = 1
            else:
                self.q_sol = self.q_before 
                response.q0sol.data = float(self.q_before[0])
                response.q1sol.data = float(self.q_before[1])
                response.q2sol.data = float(self.q_before[2])
                response.ipk.data = 2
        elif mode == 1:
            self.ref_mode = int(request.x.data)
            self.get_logger().info(f'Received Mode 1: ref_mode = {self.ref_mode}')
            response.result.data = 1
            self.mode = 1
        elif mode == 2:
            response.result.data = 1
            self.get_logger().info(f'Received Mode 2: AUTO')
            self.mode = 2
            if self.new_target:
                request.newtarget.data = 1
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModeServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
