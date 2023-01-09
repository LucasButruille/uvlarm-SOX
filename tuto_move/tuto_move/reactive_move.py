import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud

class AutoRobot(Node):
    def __init__(self):
        super().__init__('move')
        #self.topic = '/multi/cmd_nav'
        self.topic = 'cmd_vel'
        self.create_subscription( PointCloud, 'obstacles', self.avoid_obstacles, 10)
        self.velocity_publisher = self.create_publisher(Twist, self.topic, 10)
        # self.lin = (float)(lin)
        # self.ang = (float)(ang)

    def avoid_obstacles(self, pntcld) :
        obstacles = pntcld.points
        self.velo = Twist()
        sampleright = 0
        sampleleft = 0
        for point in obstacles :
            if point.y >= 0 :
                sampleright += 1
            else :
                sampleleft += 1

        print(f"Left : {sampleleft} | Right : {sampleright}")
        if (sampleleft > sampleright) :
            self.velo.linear.x = 0.0
            self.velo.angular.z = -0.4
        elif (sampleleft < sampleright) :
            self.velo.linear.x = 0.0
            self.velo.angular.z = 0.4
        else :
            self.velo.linear.x = 0.2
            self.velo.angular.z = 0.0
    
        self.velocity_publisher.publish(self.velo)

def reactive_move(args=None) :
    rclpy.init(args=args)
    autorobot = AutoRobot()
    # Start the ros infinit loop with the move node.
    rclpy.spin(autorobot)
    autorobot.destroy_node()
    rclpy.shutdown()