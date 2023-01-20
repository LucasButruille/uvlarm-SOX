import cv2
import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class Vision(Node):

    def __init__(self):
        super().__init__('Vision')
        self.camera_subscript = self.create_subscription(Image, 'img_publish', self.traitement, 10)
        self.detection = self.create_publisher(Bool, '/detection_objet', 10)
        self.lo = np.array([7, 200, 170])
        self.hi = np.array([30, 255,255])
        self.color_info = (0, 0, 255)
        self.hsv_px = [0,0,0]
        self.kernel = np.ones((7, 7), np.uint8)
        self.bridge = CvBridge()
        self.objet = Bool()
        self.rapport1 = 0
        self.rapport2 = 0

    def traitement(self, cam):  

        def souris(event, x, y, flags, param):
            if event == cv2.EVENT_MOUSEMOVE:
                # Conversion des trois couleurs RGB sous la souris en HSV
                px = frame[y,x]
                px_array = np.uint8([[px]])
                self.hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)
        
        cv2.namedWindow("Camera")
        cv2.setMouseCallback("Camera", souris)

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
        pixel_hsv = " ".join(str(values) for values in self.hsv_px)
        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            x_middle, y_middle = x+w/2, y+h/2
            self.rapport1 = round(h/w, 1)
            self.rapport2 = round(w/h, 1)
            cv2.rectangle(image2, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.circle(frame, (int(x_middle), int(y_middle)), 5, self.color_info, -1)
            cv2.circle(image2, (int(x_middle), int(y_middle)), 5, self.color_info, -1)
            cv2.putText(frame, "Objet !!!", (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
            cv2.putText(image2, "Objet !!!", (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
            cv2.putText(frame, f"x : {x_middle}, y : {y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
            cv2.putText(image2, f"x : {x_middle}, y : {y_middle}", (10,30), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
            # cv2.putText(frame, f"rapport : {int(h/w)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)
            # cv2.putText(image2, f"rapport : {round(h/w, 2)}", (10,60), cv2.FONT_HERSHEY_DUPLEX, 0.8, self.color_info, 1, cv2.LINE_AA)


        # cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Camera", frame)
        cv2.imshow('image2', image2)
        cv2.waitKey(1)

        if (self.rapport1 > 1.5 and self.rapport1 < 3) or (self.rapport2 > 1.5 and self.rapport2 < 3):
            self.objet.data = True
        else:
            self.objet.data = False
    
        self.detection.publish(self.objet)

def main(args=None):
    rclpy.init(args=args)
    Image = Vision()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(Image)
    Image.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
