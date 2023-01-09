import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud

class AutoRobot(Node):
    def __init__(self):
        super().__init__('move')
        self.topic = '/multi/cmd_nav'
        self.create_subscription( PointCloud, 'obstacles', self.avoid_obstacles, 10)
        self.velocity_publisher = self.create_publisher(Twist, self.topic, 10)
        # self.timer = self.create_timer(0.1, self.avoid_obstacles)
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

        print('sampleleft : ' + (str)(sampleleft) + ' sampleright :' + (str)(sampleright))
        if (sampleleft > sampleright and sampleleft >= 30 and sampleleft < 80) : # Tourner à droite zone 1
            self.velo.linear.x = 0.2
            self.velo.angular.z = 0.6
        elif (sampleleft < sampleright and sampleright >= 30 and sampleright < 80) : # Tourner à gauche zone 2
            self.velo.linear.x = 0.2
            self.velo.angular.z = -0.6
        elif (sampleleft > sampleright and sampleright >= 80 and sampleleft < 120) : # Tourner à droite zone 3
            self.velo.linear.x = 0.1
            self.velo.angular.z = 0.6
        elif (sampleleft < sampleright and sampleright >= 80 and sampleright < 120) : # Tourner à gauche zone 4
            self.velo.linear.x = 0.1
            self.velo.angular.z = -0.6
        elif (sampleleft > sampleright and sampleright >= 120) : # Tourner à droite zone 5
            self.velo.linear.x = 0.0
            self.velo.angular.z = 0.6
        elif (sampleleft < sampleright and sampleright >= 120) : # Tourner à gauche zone 6
            self.velo.linear.x = 0.0
            self.velo.angular.z = -0.6
        else :
            self.velo.linear.x = 0.4
            self.velo.angular.z = 0.0
    
        self.velocity_publisher.publish(self.velo)

def reactive_move(args=None) :
    rclpy.init(args=args)
    autoRobot = AutoRobot()
    # Start the ros infinit loop with the move node.
    rclpy.spin(autoRobot)
    autoRobot.destroy_node()
    rclpy.shutdown()