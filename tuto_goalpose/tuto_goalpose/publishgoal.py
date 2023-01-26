#!python3
import time, rclpy
from rclpy.node import Node
import tf2_ros.buffer, tf2_ros.transform_listener
# from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose, PoseStamped
import PyKDL
from std_msgs.msg import Float64, Bool
from visualization_msgs.msg import MarkerArray, Marker


class PoseTransformer(Node):
    def __init__(self, name, reference_frame='base_link', target_frame='map'):
        super().__init__( name )
        # Transform tool:
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        # Node Attribute:
        self.reference_frame= reference_frame

        self.posePublisher= self.create_publisher(PoseStamped, 'local_goal', 10)
        self.robotpose = self.create_publisher(PoseStamped, '/robot_pose', 10)
        # self.create_subscription(Pose, '/pose_to_transform', self.publish_pose, 10)
        self.pose= Pose()
        self.target_frame= target_frame

        self.pub_marker_rviz = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.pub_markerarray = self.create_publisher(MarkerArray, '/marker_transform', 10)

        self.create_subscription(Float64, '/distance', self.get_distance, 10)
        # self.create_subscription(PointStamped, '/clicked_point', self.get_point, 10)
        self.create_subscription(Bool, '/bottleOK', self.is_bottle, 10)
        self.marker_rviz = MarkerArray()
        self.marker_tra = MarkerArray()
        # self.timer = self.create_timer(0.1, self.pub_robot_pose)
        self.ID = 0
        self.distance = (float)(0.0)
        self.bottle = False   
        

    # def activate_publisher(self):
    #     # Local Pose Publisher:
    #     self.posePublisher= self.create_publisher(PoseStamped, 'local_goal', 10)
    #     self.create_timer(0.1, self.publish_pose)

    def getTransform(self):
        currentTime= rclpy.time.Time().to_msg()
        stampedTransform= None
        # Get Transformation
        try:
            stampedTransform= self.tf_buffer.lookup_transform(
                        self.target_frame,
                        self.reference_frame,
                        currentTime)
        except tf2_ros.TransformException as tex:
            self.get_logger().info( f'Could not transform the Pose from "{self.reference_frame}" into "{self.target_frame}": {tex}')
            return
        return stampedTransform


    def transform_to_kdl(self,t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                    t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x,
                                        t.transform.translation.y,
                                        t.transform.translation.z))
    
    def do_transform_pose(self, pose, transform):
        f = self.transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                            pose.pose.orientation.z, pose.pose.orientation.w),
                                                    PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        res = PoseStamped()
        res.pose.position.x = f[(0, 3)]
        res.pose.position.y = f[(1, 3)]
        res.pose.position.z = f[(2, 3)]
        (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
        res.header = transform.header
        return res


    def transform_pose(self, p):
        stampedTransform= self.getTransform()
        # Compute goal in local coordinates
        stampedGoal= PoseStamped()
        self.pose.position.x= p.position.x
        self.pose.position.y= p.position.y
        stampedGoal.pose= self.pose
        stampedGoal.header.frame_id= self.target_frame
        stampedGoal.header.stamp= stampedTransform.header.stamp
        localGoal = self.do_transform_pose( stampedGoal, stampedTransform )
        # # Publish
        # self.posePublisher.publish(localGoal)
        return localGoal

    
    def is_bottle(self, val) :
        self.bottle = val.data
        print(self.bottle)

    def get_distance(self, dist):
        if self.bottle:
            print("2")
            self.x = dist.data
            self.y = 0.0
            self.z = 0.0

            self.marker = Marker()
            self.marker.header.stamp = rclpy.time.Time().to_msg()
            self.marker.type = 3
            self.marker.action = 0
            self.marker.id = self.ID
            self.marker.header.frame_id = 'map'
            self.marker.pose.position.x = self.x
            self.marker.pose.position.y = self.y
            self.marker.pose.position.z = self.z
            newp = self.transform_pose(self.marker.pose)
            self.marker.pose = newp.pose
            self.marker.color.a = 1.0
            self.marker.color.r = (float)(255/255)
            self.marker.color.g = (float)(128/255)
            self.marker.color.b = (float)(0.0)
            self.marker.scale.x = 0.1
            self.marker.scale.y = 0.1
            self.marker.scale.z = 0.2

            self.marker_rviz.markers.append(self.marker)

            # self.marker_tra.markers.append(self.marker)

            self.pub_marker_rviz.publish(self.marker_rviz)
            # self.pub_markerarray.publish(self.marker_tra)

            self.ID += 1
            self.bottle = False

    # def pub_robot_pose(self):
    #     p = PoseStamped()
    #     p.pose.position.x = 0.0
    #     p.pose.position.y = 0.0
    #     newp = self.transform_pose(p.pose)
    #     self.robotpose.publish(newp)




        

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Initialize ScanInterperter
    node = PoseTransformer('goal_keeper')
    
    # infinite Loop
    rclpy.spin(node)
    
    # Clean end
    node.destroy_node()
    rclpy.shutdown()
