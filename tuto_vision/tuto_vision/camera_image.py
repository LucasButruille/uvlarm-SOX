#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import time, numpy as np
import sys, cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera(Node) :

    def __init__(self):
        super().__init__('CamNode')
        self.image_publisher = self.create_publisher(Image, 'img_publish', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_publish', 10)
        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)
        self.timer = self.create_timer(0.1, self.pub_cam)

    def pub_cam(self) :

        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        print( f"Connect: {device_product_line}" )
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)

        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
        config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)

        # Start streaming
        pipeline.start(config)

        try:
            count= 1
            refTime= time.process_time()
            freq= 60

            sys.stdout.write("-")

            while True:

                # Wait for a coherent tuple of frames: depth, color and accel
                frames = pipeline.wait_for_frames()

                depth_frame = frames.first(rs.stream.depth)
                color_frame = frames.first(rs.stream.color)

                if not (depth_frame and color_frame):
                    continue
                
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

                # Show images
                images = np.hstack((color_image, depth_colormap)) # supose that depth_colormap_dim == color_colormap_dim (640x480) otherwize: resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)
                
                # Frequency:
                if count == 10 :
                    newTime= time.process_time()
                    freq= 10/((newTime-refTime))
                    refTime= newTime
                    count= 0
                count+= 1

                # Publishing
                self.bridge = CvBridge()

                # Camera image
                msg_image = self.bridge.cv2_to_imgmsg(color_image,"bgr8")
                msg_image.header.stamp = self.get_clock().now().to_msg()
                msg_image.header.frame_id = "image"
                self.image_publisher.publish(msg_image)

                # Depth image
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
                msg_depth.header.stamp = msg_image.header.stamp
                msg_depth.header.frame_id = "depth"
                self.depth_publisher.publish(msg_depth)

                # Infra image
                infra_frame_1 = frames.get_infrared_frame(1)
                infra_frame_2 = frames.get_infrared_frame(2)
                infra_image_1 = np.asanyarray(infra_frame_1.get_data())
                infra_image_2 = np.asanyarray(infra_frame_2.get_data())

                infra_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_1, alpha=0.03), cv2.COLORMAP_JET)
		
                # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
                infra_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_2, alpha=0.03), cv2.COLORMAP_JET)	
                
                msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_1,"bgr8")
                msg_infra.header.stamp = msg_image.header.stamp
                msg_infra.header.frame_id = "infrared_1"
                self.infra_publisher_1.publish(msg_infra)
                
                msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_2,"bgr8")
                msg_infra.header.stamp = msg_image.header.stamp
                msg_infra.header.frame_id = "infrared_2"
                self.infra_publisher_2.publish(msg_infra)

        finally:
            # Stop streaming
            print("\nEnding...")
            pipeline.stop()

def main(args=None) :
    rclpy.init(args=args)
    cam = Camera()

    while rclpy.ok() :
        # Start the ros infinit loop with the move node.
        rclpy.spin_once(cam, timeout_sec = 0.1)

    # At the end, destroy the node explicitly.
    cam.destroy_node()

    # and shut the light down.
    rclpy.shutdown()