import cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Vision(Node):

    def __init__(self):
        super().__init__('Vision')
        self.camera_subscript = self.create_subscription(Image, 'img_publish', self.traitement, 10)
        cv2.namedWindow("Camera")
        self.bridge = CvBridge()
        

    def control(self,x):
        self.lo[0] = int(cv2.getTrackbarPos('Hmin', "Controls"))
        self.hi[0] = int(cv2.getTrackbarPos('Hmax', "Controls"))
        self.lo[1] = int(cv2.getTrackbarPos('Smin', "Controls"))
        self.hi[1] = int(cv2.getTrackbarPos('Smax', "Controls"))
        self.lo[2] = int(cv2.getTrackbarPos('Vmin', "Controls"))
        self.hi[2] = int(cv2.getTrackbarPos('Vmax', "Controls"))

    def traitement(self, cam):          

        frame = self.bridge.imgmsg_to_cv2(cam, desired_encoding='passthrough')
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)
        


def main(args=None):
    rclpy.init(args=args)
    Image = Vision()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(Image)
    Image.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
