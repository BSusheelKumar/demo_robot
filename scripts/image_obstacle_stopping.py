#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class TwistToTwistStampedYOLOv8(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped_yolo_v8')

        self.yolo = YOLO('yolov8s.pt') # using yolo v8 model
        self.bridge = CvBridge()


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


        # subscribing to image topic for image processing

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )


        # publishing the velocities where the robot listens
        self.twist_stamped_pub = self.create_publisher(
            TwistStamped,
            '/diff_cont/cmd_vel',
            10
        )

        self.obstacle_detected = False 


    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # converting ros2 image to cv2 image because ros2 image topic publishes different type of image and cv2 expects different type of image data

        results = self.yolo(frame)

        # predicting the results from the image data
        for result in results:
            for box in result.boxes:
                if box.conf[0] > 0.4:  
                    self.obstacle_detected = True
                    self.get_logger().info(f'Obstacle detected: {result.names[int(box.cls[0])]}')
                    self.stop_robot()
                    return 

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
    node = TwistToTwistStampedYOLOv8()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
