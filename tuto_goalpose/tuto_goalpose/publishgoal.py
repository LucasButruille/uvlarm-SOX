#!python3
import time, rclpy
from rclpy.node import Node
import tf2_ros.buffer, tf2_ros.transform_listener
# from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose, PoseStamped
import PyKDL

class PoseTransformer(Node):
    def __init__(self, name, reference_frame='map', target_frame='base_link'):
        super().__init__( name )
        # Transform tool:
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        # Node Attribute:
        self.reference_frame= reference_frame

        self.posePublisher= self.create_publisher(PoseStamped, 'local_goal', 10)
        self.create_subscription(Pose, '/pose_to_transform', self.publish_pose, 10)
        self.pose= Pose()
        self.target_frame= target_frame       
        

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


    def publish_pose(self, p):
        stampedTransform= self.getTransform()
        # Compute goal in local coordinates
        stampedGoal= PoseStamped()
        self.pose.position.x= p.position.x
        self.pose.position.y= p.position.y
        stampedGoal.pose= self.pose
        stampedGoal.header.frame_id= self.target_frame
        stampedGoal.header.stamp= stampedTransform.header.stamp
        localGoal = self.do_transform_pose( stampedGoal, stampedTransform )
        # Publish
        self.posePublisher.publish(localGoal)

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Initialize ScanInterperter
    node = PoseTransformer('goal_keeper')
    node.activate_publisher()
    
    # infinite Loop
    rclpy.spin(node)
    
    # Clean end
    node.destroy_node()
    rclpy.shutdown()
