#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import sys, cv2, math, time
from rclpy.node import Node
import rclpy
from std_msgs.msg import UInt64MultiArray, Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera(Node) :

    def __init__(self):
        super().__init__('CamNode')
        self.image_publisher = self.create_publisher(Image, 'img_publish', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_publish', 10)
        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)

        self.get_coordonnee = self.create_subscription(UInt64MultiArray, '/coordonnee_objet', self.get_distance, 10)
        self.distance_publisher = self.create_publisher(Float64MultiArray, '/distance', 10)

        self.timer = self.create_timer(0.1, self.pub_cam)

        self.x = 0
        self.y = 0
        self.dist = Float64MultiArray()

    

    def get_distance(self, coord):
        self.x = coord.data[0]
        self.y = coord.data[1]


    def pub_cam(self) :

        # self.Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        colorizer = rs.colorizer()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        device = self.pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        print( f"Connect: {device_product_line}" )
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera required !!!")
            exit(0)

        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
        self.config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)


        # Start streaming
        self.pipeline.start(self.config)

        try:

            align_to = rs.stream.depth
            align = rs.align(align_to)

            color_info=(0, 0, 255)
            rayon=10
            
            count= 1
            refTime= time.process_time()
            freq= 60

            sys.stdout.write("-")

            while True:

                # Wait for a coherent tuple of frames: depth, color and accel
                frames = self.pipeline.wait_for_frames()

                #Aligning color frame to depth frame
                aligned_frames =  align.process(frames)
                depth_frame2 = aligned_frames.get_depth_frame()
                aligned_color_frame = aligned_frames.get_color_frame()

                depth_frame = frames.first(rs.stream.depth)
                color_frame = frames.first(rs.stream.color)

                if not (depth_frame and depth_frame2 and color_frame and aligned_color_frame):
                    continue

                ## Distance ##
                colorized_depth = colorizer.colorize(depth_frame2)
                depth_colormap = np.asanyarray(colorized_depth.get_data())

                # Get the intrinsic parameters
                color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics

                color_image = np.asanyarray(aligned_color_frame.get_data())

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                #Use pixel value of  depth-aligned color image to get 3D axes
                tab_dist = []
                for y in range(0,color_colormap_dim[0],10):                                     # Hauteur
                    for x in range(0,color_colormap_dim[1],10):                                 # Largeur
                        # x, y = int(color_colormap_dim[1]/2), int(color_colormap_dim[0]/2)
                        depth = depth_frame2.get_distance(x, y)
                        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
                        distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
                        tab_dist.append(round(distance,2))
                
                self.dist.data = tab_dist
                self.distance_publisher.publish(self.dist)


                # #Show images
                # x, y = int(color_colormap_dim[1]/2), int(color_colormap_dim[0]/2)
                # depth = depth_frame2.get_distance(x, y)
                # dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
                # distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))

                # # images = np.hstack((color_image, depth_colormap)) # supose that depth_colormap_dim == color_colormap_dim (640x480) otherwize: resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                # images = color_image
                # cv2.circle(images, (int(x), int(y)), int(rayon), color_info, 2)
                # cv2.circle(images, (int(x+color_colormap_dim[1]), int(y)), int(rayon), color_info, 2)
                
                # # Affichage distance au pixel (x,y)
                # cv2.putText(images, "D="+str(round(distance,2)), (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)
                # cv2.putText(images, "D="+str(round(distance,2)), (int(x+color_colormap_dim[1])+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

                # # Show images
                # cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)

                # # Resize the Window
                # cv2.resizeWindow('RealSense', 960, 720)
                # cv2.imshow('RealSense', images)
                # cv2.waitKey(1)


                ## Normal ##
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

                # Show images
                # images = np.hstack((color_image, depth_colormap)) # supose that depth_colormap_dim == color_colormap_dim (640x480) otherwize: resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

                # Show images
                # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('RealSense', images)
                # cv2.waitKey(1)
                
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
            self.pipeline.stop()

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