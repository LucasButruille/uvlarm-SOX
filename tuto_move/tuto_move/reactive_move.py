import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2

class AutoRobot(Node):
    def __init__(self):
        super().__init__('move')
        self.topic = '/multi/cmd_nav'
        self.create_subscription( PointCloud2, 'obstacles', self.avoid_obstacles, 10)
        self.velocity_publisher = self.create_publisher(Twist, self.topic, 10)
        # self.timer = self.create_timer(0.1, self.avoid_obstacles)
        # self.lin = (float)(lin)
        # self.ang = (float)(ang)

    def avoid_obstacles(self, pntcld) :
        obstacles = pntcld.points
        self.velo = Twist()
        sampleright = []
        sampleleft = []
        for i in obstacles :
            if i.x >= 0 :
                sampleright.append(i)
            else :
                sampleleft.append(i)

        print('sampleleft : ' + len(sampleleft) + ' sampleright :' + len(sampleright))
        if (len(sampleleft) > len(sampleright)) :
            self.velo.linear.x = 0.0
            self.velo.angular.z = -0.4
        elif (len(sampleleft) < len(sampleright)) :
            self.velo.linear.x = 0.0
            self.velo.angular.z = 0.4
        else :
            self.velo.linear.x = 0.2
            self.velo.angular.z = 0.0
    
        self.velocity_publisher.publish(self.velo)

def reactive_move(args=None) :
    rclpy.init(args=args)
    autoRobot = AutoRobot()
    # Start the ros infinit loop with the move node.
    rclpy.spin(autoRobot)
    autoRobot.destroy_node()
    rclpy.shutdown()