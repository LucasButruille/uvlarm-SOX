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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class Camera(Node) :

    def __init__(self):
        super().__init__('CamNode')
        self.image_publisher = self.create_publisher(Image, 'img_publish', 10)
        # self.depth_publisher = self.create_publisher(Image, 'depth_publish', 10)
        # self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        # self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)
        self.timer = self.create_timer(0.1, self.pub_cam)

        self.detection = self.create_publisher(String, '/detection_objet', 10)
        self.lo = np.array([7, 200, 170])
        self.hi = np.array([30, 255,255])
        self.color_info = (0, 0, 255)
        self.hsv_px = [0,0,0]
        self.kernel = np.ones((7, 7), np.uint8)
        self.bridge = CvBridge()
        self.objet = False
        self.objetmsg = String()
        self.w, self.h = 0, 0
        self.rapport1 = 0
        self.rapport2 = 0
        self.old = False
        self.x_middle = (int)(0)
        self.y_middle = (int)(0)
        self.dist_tab = []
        self.distance_moyenne = 0
        cv2.namedWindow("Controls")
        cv2.resizeWindow("Controls", 550,20)
        # cv2.setMouseCallback("Camera", souris)
        cv2.createTrackbar('Hmin', "Controls" , 7, 50, self.control)
        cv2.createTrackbar('Hmax', "Controls" , 30, 50, self.control)
        cv2.createTrackbar('Smin', "Controls" , 200, 255, self.control)
        cv2.createTrackbar('Smax', "Controls" , 255, 255, self.control)
        cv2.createTrackbar('Vmin', "Controls" , 170, 255, self.control)
        cv2.createTrackbar('Vmax', "Controls" , 255, 255, self.control)

    def control(self,x):
        self.lo[0] = int(cv2.getTrackbarPos('Hmin', "Controls"))
        self.hi[0] = int(cv2.getTrackbarPos('Hmax', "Controls"))
        self.lo[1] = int(cv2.getTrackbarPos('Smin', "Controls"))
        self.hi[1] = int(cv2.getTrackbarPos('Smax', "Controls"))
        self.lo[2] = int(cv2.getTrackbarPos('Vmin', "Controls"))
        self.hi[2] = int(cv2.getTrackbarPos('Vmax', "Controls"))

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
        # self.config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
        # self.config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)


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

                

                depth_frame = frames.first(rs.stream.depth)
                color_frame = frames.first(rs.stream.color)

                if not (depth_frame and color_frame):
                    continue


                #### Distance ####

                # #Aligning color frame to depth frame
                # aligned_frames =  align.process(frames)
                # depth_frame2 = aligned_frames.get_depth_frame()
                # aligned_color_frame = aligned_frames.get_color_frame()

                depth_frame2 = frames.get_depth_frame()
                aligned_color_frame = frames.get_color_frame()

                if not (depth_frame2 and aligned_color_frame):
                    continue

                colorized_depth = colorizer.colorize(depth_frame2)
                depth_colormap = np.asanyarray(colorized_depth.get_data())

                # Get the intrinsic parameters
                color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
                color_image2 = np.asanyarray(aligned_color_frame.get_data())

                ## Normal ##
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

                

                # # Depth image
                # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                # msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
                # msg_depth.header.stamp = msg_image.header.stamp
                # msg_depth.header.frame_id = "depth"
                # self.depth_publisher.publish(msg_depth)

                # # Infra image
                # infra_frame_1 = frames.get_infrared_frame(1)
                # infra_frame_2 = frames.get_infrared_frame(2)
                # infra_image_1 = np.asanyarray(infra_frame_1.get_data())
                # infra_image_2 = np.asanyarray(infra_frame_2.get_data())

                # infra_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_1, alpha=0.03), cv2.COLORMAP_JET)
        
                # # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
                # infra_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_2, alpha=0.03), cv2.COLORMAP_JET)	
                
                # msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_1,"bgr8")
                # msg_infra.header.stamp = msg_image.header.stamp
                # msg_infra.header.frame_id = "infrared_1"
                # self.infra_publisher_1.publish(msg_infra)
                
                # msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_2,"bgr8")
                # msg_infra.header.stamp = msg_image.header.stamp
                # msg_infra.header.frame_id = "infrared_2"
                # self.infra_publisher_2.publish(msg_infra)

                # time.sleep(0.5)


                ###### TRAITEMENT ######

                frame = color_image
                image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                # Flouttage de l'image
                image=cv2.blur(image, (7, 7))
                # Recherche de la couleur
                mask=cv2.inRange(image, self.lo, self.hi)
                # Ouverture / Fermeture
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
                image2=cv2.bitwise_and(frame, frame, mask = mask)
                # pixel_hsv = " ".join(str(values) for values in self.hsv_px)
                elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                if len(elements) > 0:
                    c=max(elements, key=cv2.contourArea)
                    x,y,self.w,self.h = cv2.boundingRect(c)
                    self.x_middle, self.y_middle = int(x+self.w/2), int(y+self.h/2)
                    self.rapport1 = round(self.h/self.w, 1)
                    self.rapport2 = round(self.w/self.h, 1)
                    if (self.rapport1 > 1.5 and self.rapport1 < 3) or (self.rapport2 > 1.5 and self.rapport2 < 3):
                        self.objet = True
                    else:
                        self.objet = False
                        self.objetmsg.data = "Pas de bouteille"

                    if self.objet:
                        # Get distance
                        depth = depth_frame2.get_distance(self.x_middle, self.y_middle)
                        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [self.x_middle,self.y_middle], depth)
                        distance = round(math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2)),2)
                        
                        if distance < 2.0 and distance > 0.1:
                            self.dist_tab.append(distance)

                        if len(self.dist_tab)>20:
                            self.distance_moyenne = round(np.mean(self.dist_tab),2)
                            self.dist_tab = []

                        # Rectangle autour de la bouteille
                        cv2.rectangle(image2, (x, y), (x+self.w, y+self.h), (0, 0, 255), 2)
                        cv2.rectangle(frame, (x, y), (x+self.w, y+self.h), (0, 0, 255), 2)
                        # Point au milieu de la bouteille
                        cv2.circle(frame, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                        cv2.circle(image2, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                        # Affichage de la distance
                        cv2.putText(frame, str(self.distance_moyenne) + 'm', (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                        cv2.putText(image2, str(self.distance_moyenne) + 'm', (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                        
                        self.objetmsg.data = f"Bouteille à {self.distance_moyenne}m."

                        # Coordonnées de la bouteille
                        # cv2.putText(frame, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                        # cv2.putText(image2, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                        # Affichage du rapport
                        # cv2.putText(frame, f"rapport : {int(h/w)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                        # cv2.putText(image2, f"rapport : {round(h/w, 2)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)


                # cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                # cv2.imshow("Camera", frame)
                cv2.imshow('Mask', image2)
                cv2.waitKey(1)

                if self.objet != self.old:
                    self.detection.publish(self.objetmsg)

                self.old = self.objet


                # Publishing
                self.bridge = CvBridge()

                # Camera image
                msg_image = self.bridge.cv2_to_imgmsg(frame,"bgr8")
                msg_image.header.stamp = self.get_clock().now().to_msg()
                msg_image.header.frame_id = "image"
                self.image_publisher.publish(msg_image)


                # Frequency:
                if count == 10 :
                    newTime= time.process_time()
                    freq= 10/((newTime-refTime))
                    refTime= newTime
                    count= 0
                count+= 1


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