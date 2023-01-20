import cv2
import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt64MultiArray, Float64MultiArray
from cv_bridge import CvBridge


class Vision(Node):

    def __init__(self):
        super().__init__('Vision')
        self.camera_subscript = self.create_subscription(Image, 'img_publish', self.traitement, 10)
        self.distance_sub = self.create_subscription(Float64MultiArray, '/distance', self.get_distance, 10)
        self.detection = self.create_publisher(Bool, '/detection_objet', 10)
        self.coordonnee = self.create_publisher(UInt64MultiArray, '/coordonnee_objet', 10)
        self.lo = np.array([7, 200, 170])
        self.hi = np.array([30, 255,255])
        self.color_info = (0, 0, 255)
        self.hsv_px = [0,0,0]
        self.kernel = np.ones((7, 7), np.uint8)
        self.bridge = CvBridge()
        self.objet = Bool()
        self.coord = UInt64MultiArray()
        self.rapport1 = 0
        self.rapport2 = 0
        self.old = False
        self.x_middle = (int)(0)
        self.y_middle = (int)(0)
        self.distance = (float)(0.0)
        cv2.namedWindow("Camera")
        cv2.namedWindow("Controls")
        cv2.resizeWindow("Controls", 550,10)
        # cv2.setMouseCallback("Camera", souris)
        cv2.createTrackbar('Hmin', "Controls" , 7, 50, self.control)
        cv2.createTrackbar('Hmax', "Controls" , 30, 50, self.control)
        cv2.createTrackbar('Smin', "Controls" , 200, 255, self.control)
        cv2.createTrackbar('Smax', "Controls" , 255, 255, self.control)
        cv2.createTrackbar('Vmin', "Controls" , 170, 255, self.control)
        cv2.createTrackbar('Vmax', "Controls" , 255, 255, self.control)

    def get_distance(self, dist):

        if (self.x_middle-5)//10 == (self.x_middle)//10: 
            x = ((self.x_middle)//10 + 1)
        else :
            x = ((self.x_middle)//10)

        if (self.y_middle-5)//10 == (self.y_middle)//10: 
            y = ((self.y_middle)//10 + 1)
        else :
            y = ((self.y_middle)//10)

        # print('x : ' + str(x))
        # print('y : ' + str(y))

        self.distance = dist.data[int(x*y)]

    def control(self,x):
        self.lo[0] = int(cv2.getTrackbarPos('Hmin', "Controls"))
        self.hi[0] = int(cv2.getTrackbarPos('Hmax', "Controls"))
        self.lo[1] = int(cv2.getTrackbarPos('Smin', "Controls"))
        self.hi[1] = int(cv2.getTrackbarPos('Smax', "Controls"))
        self.lo[2] = int(cv2.getTrackbarPos('Vmin', "Controls"))
        self.hi[2] = int(cv2.getTrackbarPos('Vmax', "Controls"))

    def traitement(self, cam):  

        # def souris(event, x, y, flags, param):
        #     if event == cv2.EVENT_MOUSEMOVE:
        #         # Conversion des trois couleurs RGB sous la souris en HSV
        #         px = frame[y,x]
        #         px_array = np.uint8([[px]])
        #         self.hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)
        

        frame = self.bridge.imgmsg_to_cv2(cam, desired_encoding='passthrough')
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
            x,y,w,h = cv2.boundingRect(c)
            self.x_middle, self.y_middle = int(x+w/2), int(y+h/2)
            self.rapport1 = round(h/w, 1)
            self.rapport2 = round(w/h, 1)

            if (self.rapport1 > 1.5 and self.rapport1 < 3) or (self.rapport2 > 1.5 and self.rapport2 < 3):
                self.objet.data = True
            else:
                self.objet.data = False

            if self.objet.data:
                cv2.rectangle(image2, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.circle(frame, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                cv2.circle(image2, (int(self.x_middle), int(self.y_middle)), 5, self.color_info, -1)
                cv2.putText(frame, str(self.distance), (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                cv2.putText(image2, str(self.distance), (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                cv2.putText(frame, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                cv2.putText(image2, f"x : {self.x_middle}, y : {self.y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                # cv2.putText(frame, f"rapport : {int(h/w)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
                # cv2.putText(image2, f"rapport : {round(h/w, 2)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)

        # cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Camera", frame)
        cv2.imshow('image2', image2)
        cv2.waitKey(1)

        if self.objet.data != self.old:
            self.detection.publish(self.objet)

        self.old = self.objet.data

        self.coord.data = [self.x_middle, self.y_middle]
        if self.coord.data != [0,0]:
            self.coordonnee.publish(self.coord)


def main(args=None):
    rclpy.init(args=args)
    Image = Vision()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(Image)
    Image.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()