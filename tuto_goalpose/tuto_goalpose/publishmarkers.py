import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PointStamped

class LocalGoal(Node):
    def __init__(self):
        super().__init__('goal_keeper')
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.subpoint = self.create_subscription(PointStamped, '/clicked_point', self.get_point, 10)
        self.markerarray = MarkerArray()
        self.timer = self.create_timer(0.1, self.pub_marker)
        self.ID = 0

    def get_point(self, pnt):

        self.marker = Marker()
        self.x = pnt.point.x
        self.y = pnt.point.y
        self.z = pnt.point.z
        self.marker.type = 3
        self.marker.action = 0
        self.marker.id = self.ID
        self.marker.header.frame_id = 'map'
        self.marker.pose.position.x = self.x
        self.marker.pose.position.y = self.y
        self.marker.pose.position.z = self.z
        self.marker.color.a = 1.0
        self.marker.color.r = (float)(255/255)
        self.marker.color.g = (float)(128/255)
        self.marker.color.b = (float)(0.0)
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.4

        self.markerarray.markers.append(self.marker)

        self.ID += 1

    def pub_marker(self):
        self.publisher.publish(self.markerarray)
        


def main(args=None) :
    rclpy.init(args=args)
    goalpose = LocalGoal()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(goalpose)
    goalpose.destroy_node()
    rclpy.shutdown()