class LocalGoal(Node):
    def __init__(self):
        super().__init__('goal_keeper')
        # Transform tool:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Node Attribute:
        self.local_frame= 'base_link'
        self.global_goal= Pose()
        self.global_goal.position.x= (float)(1)
        self.global_goal.position.y= (float)(2)
        # Local Goal Publisher:
        self.create_timer(0.1, self.publish_goal)

    def publish_goal(self):
        currentTime= rclpy.time.Time()
        # Get Transformation
        try:
            stampedTransform= self.tf_buffer.lookup_transform(
                        self.local_frame,
                        'odom',
                        currentTime)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):TransformException as tex:
            self.get_logger().info( f'Could not transform the goal into {self.local_frame}: {tex}')
            return