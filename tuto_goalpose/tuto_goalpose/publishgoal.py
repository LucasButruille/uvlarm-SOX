#!python3
import time, rclpy
from rclpy.node import Node
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped

class PoseTransformer(Node):
    def __init__(self, name, reference_frame='map', target_frame='base_link' ):
        super().__init__( name )
        # Transform tool:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Node Attribute:
        self.reference_frame= reference_frame
        self.pose= Pose()
        self.pose.position.x= (float)(0)
        self.pose.position.y= (float)(0)
        self.target_frame= target_frame

    def activate_publisher(self):
        # Local Pose Publisher:
        self.posePublisher= self.create_publisher(PoseStamped, 'local_goal', 10)
        self.create_timer(0.1, self.publish_pose)

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

    def publish_pose(self):
        stampedTransform= self.getTransform()
        # Compute goal in local coordinates
        stampedGoal= PoseStamped()
        stampedGoal.pose= self.pose
        stampedGoal.header.frame_id= self.target_frame
        stampedGoal.header.stamp= stampedTransform.header.stamp
        localGoal = tf2_geometry_msgs.do_transform_pose( stampedGoal, stampedTransform )
        # Publish
        self.posePublisher.publish(localGoal)

# def process_test(args=None):
#     # Initialize ROS
#     rclpy.init(args=args)

#     # Initialize ScanInterperter
#     node = PoseTransformer('tf_tester',
#         reference_frame='map',
#         target_frame='laser'
#     )
#     node.create_timer( 0.1, node.getTransform )
    
#     # infinite Loop
#     rclpy.spin(node)
    
#     # Clean end
#     node.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Initialize ScanInterperter
    node = PoseTransformer('goal_keeper')
    node.activatePublisher('local_goal')
    
    # infinite Loop
    rclpy.spin(node)
    
    # Clean end
    node.destroy_node()
    rclpy.shutdown()
