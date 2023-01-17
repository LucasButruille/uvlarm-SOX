#!/usr/bin/env python3
import sys, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
from cv_bridge import CvBridge



class Take_Picture(Node) :

    def __init__(self) :
        super().__init__('Photos')
        self.create_subscription(Image, 'img_publish', self.enregistrer, 10)
        self.count = 1
        self.bridge = CvBridge()

    def enregistrer(self, img) :
        img_final = self.bridge.imgmsg_to_cv2(img,"bgr8")
        cv2.imwrite('/home/bot/ros2_ws/opencv-haar-classifier-training/negative_images/'+str(self.count)+'.jpg', img_final)
        self.count += 1

def photo(args=None) :
    rclpy.init(args=args)
    Pic = Take_Picture()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(Pic)
    Pic.destroy_node()
    rclpy.shutdown()
