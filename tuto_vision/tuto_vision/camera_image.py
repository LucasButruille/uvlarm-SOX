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
from std_msgs.msg import Bool, Float64, Int64
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

        self.detection = self.create_publisher(Bool, '/detection_objet', 10)
        self.distancepub = self.create_publisher(Float64, '/distance', 10)
        self.bottlepospub = self.create_publisher(Int64, '/bottle_position', 10)
        self.loHSV = np.array([7, 200, 170])
        self.hiHSV = np.array([30, 255,255])
        self.loRGB = np.array([0, 0, 0])
        self.hiRGB = np.array([10, 10, 10])
        self.seuil = 10
        self.color_info = (0, 0, 255)
        self.hsv_px = [0,0,0]
        self.kernel = np.ones((7, 7), np.uint8)
        self.bridge = CvBridge()
        self.objet = Bool()
        # self.objetmsg = String()
        self.w, self.h = 0, 0
        self.rapport1 = 0
        self.rapport2 = 0
        self.x_middle = (int)(0)
        self.y_middle = (int)(0)
        self.dist_tab = []
        self.distance_moyenne = 0
        self.dist = Float64()
        self.x_bottle = Int64()
        # HSV
        cv2.namedWindow("ControlsHSV")
        cv2.resizeWindow("ControlsHSV", 550,20)
        cv2.createTrackbar('Hmin', "ControlsHSV" , 7, 50, self.ControlHSV)
        cv2.createTrackbar('Hmax', "ControlsHSV" , 30, 50, self.ControlHSV)
        cv2.createTrackbar('Smin', "ControlsHSV" , 200, 255, self.ControlHSV)
        cv2.createTrackbar('Smax', "ControlsHSV" , 255, 255, self.ControlHSV)
        cv2.createTrackbar('Vmin', "ControlsHSV" , 170, 255, self.ControlHSV)
        cv2.createTrackbar('Vmax', "ControlsHSV" , 255, 255, self.ControlHSV)
        # # RGB
        # cv2.namedWindow("ControlsRGB")
        # cv2.resizeWindow("ControlsRGB", 550,20)
        # # cv2.createTrackbar('Rmin', "ControlsRGB" , 0, 100, self.ControlRGB)
        # # cv2.createTrackbar('Rmax', "ControlsRGB" , 30, 100, self.ControlRGB)
        # # cv2.createTrackbar('Gmin', "ControlsRGB" , 0, 100, self.ControlRGB)
        # # cv2.createTrackbar('Gmax', "ControlsRGB" , 30, 100, self.ControlRGB)
        # # cv2.createTrackbar('Bmin', "ControlsRGB" , 0, 100, self.ControlRGB)
        # # cv2.createTrackbar('Bmax', "ControlsRGB" , 30, 100, self.ControlRGB)
        # cv2.createTrackbar('Seuil', "ControlsRGB" , 30, 100, self.ControlRGB)

    # def ControlRGB(self,x):
    #     self.seuil = int(cv2.getTrackbarPos('Seuil', "ControlsRGB"))
    #     # self.loRGB[0] = int(cv2.getTrackbarPos('Rmin', "ControlsRGB"))
    #     # self.hiRGB[0] = int(cv2.getTrackbarPos('Rmax', "ControlsRGB"))
    #     # self.loRGB[1] = int(cv2.getTrackbarPos('Gmin', "ControlsRGB"))
    #     # self.hiRGB[1] = int(cv2.getTrackbarPos('Gmax', "ControlsRGB"))
    #     # self.loRGB[2] = int(cv2.getTrackbarPos('Bmin', "ControlsRGB"))
    #     # self.hiRGB[2] = int(cv2.getTrackbarPos('Bmax', "ControlsRGB"))
    
    def ControlHSV(self,x):
        self.loHSV[0] = int(cv2.getTrackbarPos('Hmin', "ControlsHSV"))
        self.hiHSV[0] = int(cv2.getTrackbarPos('Hmax', "ControlsHSV"))
        self.loHSV[1] = int(cv2.getTrackbarPos('Smin', "ControlsHSV"))
        self.hiHSV[1] = int(cv2.getTrackbarPos('Smax', "ControlsHSV"))
        self.loHSV[2] = int(cv2.getTrackbarPos('Vmin', "ControlsHSV"))
        self.hiHSV[2] = int(cv2.getTrackbarPos('Vmax', "ControlsHSV"))

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
                color_image = np.asanyarray(aligned_color_frame.get_data())

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
                imageHSV=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                # imageGray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # imageGray=cv2.bitwise_not(imageGray)
                # Flouttage de l'image
                imageHSV=cv2.blur(imageHSV, (7, 7))
                # imageGray=cv2.blur(imageGray, (7, 7))
                # Recherche de la couleur
                MaskOrange=cv2.inRange(imageHSV, self.loHSV, self.hiHSV)
                # MaskNoir=cv2.inRange(imageGray, 255-self.seuil, 255)
                # Ouverture / Fermeture
                MaskOrange = cv2.morphologyEx(MaskOrange, cv2.MORPH_CLOSE, self.kernel, iterations=1)
                MaskOrange = cv2.morphologyEx(MaskOrange, cv2.MORPH_OPEN, self.kernel, iterations=1)
                ImageOrange=cv2.bitwise_and(frame, frame, mask = MaskOrange)

                # MaskNoir = cv2.morphologyEx(MaskNoir, cv2.MORPH_CLOSE, self.kernel, iterations=1)
                # MaskNoir = cv2.morphologyEx(MaskNoir, cv2.MORPH_OPEN, self.kernel, iterations=1)
                # ImageNoire=cv2.bitwise_and(imageGray, imageGray, mask = MaskNoir)
                
                # pixel_hsv = " ".join(str(values) for values in self.hsv_px)
                
                # Contours Mask Orange
                elementsOrange=cv2.findContours(MaskOrange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                if len(elementsOrange) > 0:
                    for c in elementsOrange:
                        x,y,w,h = cv2.boundingRect(c)
                        r1 = round(h/w, 1)
                        r2 = round(w/h, 1)
                        if (r1 > 1.5 and r1 < 3) or (r2 > 1.5 and r2 < 3):
                            # Rectangle autour de la bouteille
                            cv2.rectangle(ImageOrange, (x, y), (x+w, y+h), (0, 0, 255), 2)
                            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

                    c=max(elementsOrange, key=cv2.contourArea)
                    x,y,self.w,self.h = cv2.boundingRect(c)
                    self.x_middle, self.y_middle = int(x+self.w/2), int(y+self.h/2)
                    self.rapport1 = round(self.h/self.w, 1)
                    self.rapport2 = round(self.w/self.h, 1)
                    if (self.rapport1 > 1.5 and self.rapport1 < 3) or (self.rapport2 > 1.5 and self.rapport2 < 3):
                        self.objet.data = True
                    else:
                        self.objet.data = False
                        # self.objetmsg.data = "Pas de bouteille"

                    if self.objet.data:
                        self.x_bottle.data = self.x_middle
                        # Get distance
                        depth = depth_frame2.get_distance(self.x_middle, self.y_middle)
                        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [self.x_middle,self.y_middle], depth)
                        distance = round(math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2)),2)
                        
                        if distance < 2.0 and distance > 0.1:
                            self.dist_tab.append(distance)

                        if len(self.dist_tab)>20:
                            self.distance_moyenne = round(np.mean(self.dist_tab),2)
                            self.dist_tab = []

                        
                        # Point au milieu de la bouteille
                        cv2.circle(frame, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                        cv2.circle(ImageOrange, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                        # Affichage de la distance
                        cv2.putText(frame, str(self.distance_moyenne) + 'm', (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                        cv2.putText(ImageOrange, str(self.distance_moyenne) + 'm', (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                        
                        self.dist.data = (float)(self.distance_moyenne)

                        # self.objetmsg.data = f"Bouteille Orange à {self.distance_moyenne}m."

                        # Coordonnées de la bouteille
                        # cv2.putText(frame, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                        # cv2.putText(ImageOrange, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                        # Affichage du rapport
                        # cv2.putText(frame, f"rapport : {int(h/w)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                        # cv2.putText(ImageOrange, f"rapport : {round(h/w, 2)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)

                # # Contours Mask Noir
                # elementsNoir=cv2.findContours(MaskNoir, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                # if len(elementsNoir) > 0:
                #     c=max(elementsNoir, key=cv2.contourArea)
                #     x,y,self.w,self.h = cv2.boundingRect(c)
                #     self.x_middle, self.y_middle = int(x+self.w/2), int(y+self.h/2)
                #     self.rapport1 = round(self.h/self.w, 1)
                #     self.rapport2 = round(self.w/self.h, 1)
                #     if (self.rapport1 > 1.5 and self.rapport1 < 3) or (self.rapport2 > 1.5 and self.rapport2 < 3):
                #         self.objet.data = True
                #     else:
                #         self.objet.data = False
                #         # self.objetmsg.data = "Pas de bouteille"

                #     if self.objet.data:
                #         # Get distance
                #         depth = depth_frame2.get_distance(self.x_middle, self.y_middle)
                #         dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [self.x_middle,self.y_middle], depth)
                #         distance = round(math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2)),2)
                        
                #         if distance < 2.0 and distance > 0.1:
                #             self.dist_tab.append(distance)

                #         if len(self.dist_tab)>20:
                #             self.distance_moyenne = round(np.mean(self.dist_tab),2)
                #             self.dist_tab = []

                #         # Rectangle autour de la bouteille
                #         cv2.rectangle(ImageNoire, (x, y), (x+self.w, y+self.h), (0, 0, 255), 2)
                #         cv2.rectangle(frame, (x, y), (x+self.w, y+self.h), (0, 0, 255), 2)
                #         # Point au milieu de la bouteille
                #         cv2.circle(frame, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                #         cv2.circle(ImageNoire, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                #         # Affichage de la distance
                #         cv2.putText(frame, str(self.distance_moyenne) + 'm', (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                #         cv2.putText(ImageNoire, str(self.distance_moyenne) + 'm', (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                        
                #         self.dist.data = (float)(self.distance_moyenne)

                #         # self.objetmsg.data = f"Bouteille Noire à {self.distance_moyenne}m."

                #         # Coordonnées de la bouteille
                #         # cv2.putText(frame, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                #         # cv2.putText(ImageOrange, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                #         # Affichage du rapport
                #         # cv2.putText(frame, f"rapport : {int(h/w)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                #         # cv2.putText(ImageOrange, f"rapport : {round(h/w, 2)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)





                # cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("Camera", frame)
                cv2.imshow('Orange', ImageOrange)
                # cv2.imshow('Noir', ImageNoire)
                cv2.waitKey(1)


                self.detection.publish(self.objet)
                self.distancepub.publish(self.dist)
                self.bottlepospub.publish(self.x_bottle)


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