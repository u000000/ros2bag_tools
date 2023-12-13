#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np

class ExtractImagesNode(Node):
    def __init__(self):
        super().__init__('extract_ros2bag')
        self.counter = 0 
        self.bridge = cv_bridge.CvBridge()
        
        # Parameters
        self.data_folder = self.declare_parameter("data_folder_path","").value
        
        if self.data_folder == "":
            self.get_logger().error("Need folder path --ros-args --param data_folder_path:=<folder-path>")
            exit()
        
        self.image_raw_subscriber_ = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10)
        
        self.get_logger().info('Extraction has been started')
        
    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            result = cv2.imwrite(self.data_folder + '/num_' + str(self.counter) + '.png', cv_img)
            self.counter += 1
            self.get_logger().info(str(self.counter) + ' saved? ' + str(result))
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))
        

def main(args=None):
    rclpy.init(args=args)
    node = ExtractImagesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()