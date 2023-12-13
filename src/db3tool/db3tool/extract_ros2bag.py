#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv_bridge
import cv2
import numpy as np

class ExtractImagesNode(Node):
    def __init__(self):
        super().__init__('extract_ros2bag')
        self.counter = 0
                
        self.bridge = cv_bridge.CvBridge()
        
        self.image_raw_subscriber_ = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            10)
        
        self.get_logger().info('Extraction has been started')
        
    def image_callback(self, msg: CompressedImage):
        try:
            self.get_logger().info('Image being saved')
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
            cv2.imwrite('images/num_' + self.counter + '.png', cv_image)
            self.counter += 1
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))
        

def main(args=None):
    rclpy.init(args=args)
    node = ExtractImagesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()