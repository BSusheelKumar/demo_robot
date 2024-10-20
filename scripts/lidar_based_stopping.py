#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')


        # subscribing to the cmd_vel for any tele-operations from teleop twist keyboard which is not used currently

        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )


        # subscribing to the cmd_vel_nav where nav2 stack publishes velocities for navigation

        self.twist_sub_nav = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self._nav_twist_callback,
            10
        )


        # subscribing to lidar/laser scan topic for processing

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )


        # publishing the velocities where the robot listens

        self.twist_stamped_pub = self.create_publisher(
            TwistStamped,
            '/diff_cont/cmd_vel',
            10
        )

        self.safe_distance = 1.5  
        self.obstacle_detected = False  



    def laser_callback(self, msg):
        if min(msg.ranges) < self.safe_distance:
            self.get_logger().info('Obstacle detected! Stopping the robot.')
            self.obstacle_detected = True
            self.stop_robot() 
        else:
            self.obstacle_detected = False  
            self.get_logger().info('No obstacles detected.')


    # stopping the robot when obstacle is detected

    def stop_robot(self):
       
        stop_msg = TwistStamped()
        stop_msg.header = Header()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0

        self.twist_stamped_pub.publish(stop_msg)

    def _nav_twist_callback(self, twist_msg):
        if not self.obstacle_detected:
            twist_stamped_msg = TwistStamped()
            twist_stamped_msg.header = Header()
            twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            twist_stamped_msg.twist = twist_msg

            self.twist_stamped_pub.publish(twist_stamped_msg)

    def twist_callback(self, twist_msg):
        if not self.obstacle_detected:
            twist_stamped_msg = TwistStamped()
            twist_stamped_msg.header = Header()
            twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            twist_stamped_msg.twist = twist_msg

            self.twist_stamped_pub.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
