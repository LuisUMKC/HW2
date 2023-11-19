#!/usr/bin/env python3
from re import S
import rclpy
import math 
import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import some_python_module
from unmanned_systems_ros2_pkg.PIDTemplate import PID

def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9 


def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
       
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

class TurtleBotNode(Node):
    def __init__(self, ns=''):
        super().__init__('minimial_turtlebot')

        if ns != '':
            self.ns = ns
        else:
            self.ns = ns

        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  10) 

        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 10)

        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)

        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw

        print("yaw is", np.degrees(self.orientation_euler[2]))

    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)

def main()->None:
    rclpy.init(args=None)
    print("starting")

    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)

    des_x_position = 7.0
    cmd_vel = 1.5 #m/s
    ang_vel = 0.15 #rad/s
    stop_vel = 0.0
    time_duration = 12

    time_origin = get_time_in_secs(turtlebot_node)
    print("time now is", time_origin)
    kp_angular = 0.5
    ki_angular = 0.0
    kd_angular = 0.8
    dt_angular = 1/20
    pid_angular = PID(
        kp=kp_angular,
        ki=ki_angular,
        kd=kd_angular,
        dt=dt_angular
    )

    MAX_AVG_SPEED = 2.84
    waypoints = [[0,0], [0,1], [2,2], [3, -3]]
    tolerance = np.deg2rad(0.5)
    distance_error_tolreance = 0.05
    current_waypoint = 0

    rclpy.spin_once(turtlebot_node)

    while rclpy.ok():
        time_diff = get_time_in_secs(turtlebot_node) - time_origin
        current_position = (turtlebot_node.current_position[0], turtlebot_node.current_position[1])
        distance_to_waypoint = np.sqrt((current_position[0] - waypoints[current_waypoint][0]) ** 2 + (current_position[1] - waypoints[current_waypoint][1]) ** 2)

        if distance_to_waypoint < 0.15:
            current_waypoint += 1
            if current_waypoint >= len(waypoints):
                break
        print(current_waypoint)
        desired_heading = np.arctan2(waypoints[current_waypoint][1] - current_position[1], waypoints[current_waypoint][0] - current_position[0])

        angular_gains = pid_angular.get_gains(desired_heading, turtlebot_node.orientation_euler[2])
        print(f"heading error {np.rad2deg(pid_angular.error[0])}")
        print(f"angular gains {angular_gains}")

        if angular_gains > MAX_AVG_SPEED:
            angular_gains = MAX_AVG_SPEED
        elif angular_gains <= -MAX_AVG_SPEED:
            angular_gains = -MAX_AVG_SPEED

        current_error = np.abs(pid_angular.error[0])
        if current_error <= tolerance:
            turtlebot_node.move_turtle(0.0, 0.0)
            ###
            dx = waypoints[current_waypoint][0] - turtlebot_node.current_position[0]
            dy = waypoints[current_waypoint][1] -  turtlebot_node.current_position[1]
            current_distance_error = np.sqrt(dx**2 + dy**2)

            while current_distance_error >= distance_error_tolreance:
                current_heading_error_rad = pid_angular.compute_error(
                        desired_heading,
                        turtlebot_node.orientation_euler[2]                
                    )

                angular_gains = pid_angular.get_gains(
                        desired_heading,
                        turtlebot_node.orientation_euler[2]
                    )

                print("my heading error is", 
                        np.rad2deg(pid_angular.error[0]))
                print("current distance error", current_distance_error)

                if angular_gains >= MAX_AVG_SPEED:
                    angular_gains = MAX_AVG_SPEED
                elif angular_gains <= -MAX_AVG_SPEED:
                    angular_gains = -MAX_AVG_SPEED

                dx = waypoints[current_waypoint][0] - turtlebot_node.current_position[0]
                dy = waypoints[current_waypoint][1] -  turtlebot_node.current_position[1]
                current_distance_error = np.sqrt(dx**2 + dy**2)

                if (current_distance_error <= distance_error_tolreance):
                    turtlebot_node.move_turtle(0.0, 0.0)
                    break

                turtlebot_node.move_turtle(0.15, angular_gains)

                rclpy.spin_once(turtlebot_node)
            ###
        else:
            turtlebot_node.move_turtle(0.0, angular_gains)

        rclpy.spin_once(turtlebot_node)

if __name__ == '__main__':
    """apply imported function"""
    main()