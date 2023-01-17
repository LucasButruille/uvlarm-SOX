import cv2
import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Vision(Node):

    def __init__(self):
        super().__init__('Vision')
        self.camera_subscript = self.create_subscription(Image, 'img_publish', self.traitement, 10)
        self.lo = np.array([7, 180, 170])
        self.hi = np.array([30, 255,255])
        self.color_info = (0, 0, 255)
        self.hsv_px = [0,0,0]
        self.kernel = np.ones((7, 7), np.uint8)
        self.bridge = CvBridge()

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
            cv2.rectangle(image2, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
            cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)

        cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Camera", frame)
        cv2.imshow('image2', image2)
        # cv2.imshow('Mask', mask)

        cv2.waitKey(1)
        # if cv2.waitKey(1)&0xFF==ord('q'):
        #     break
    # cap.release()
    # cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    Image = Vision()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(Image)
    Image.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()