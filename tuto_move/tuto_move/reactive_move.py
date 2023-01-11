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
        # self.timer = self.create_timer(0.1, self.avoid_obstacles)
        # self.lin = (float)(lin)
        # self.ang = (float)(ang)
        self.old_lin = (float)(0.01)
        self.lin = (float)(0.1)
        self.ang = (float)(0.0)

    def avoid_obstacles(self, pntcld) :
        obstacles = pntcld.points
        self.velo = Twist()
        if obstacles != []:
            for point in obstacles :
                if point.y >= 0 and point.y < 0.5:      # Obstacles à Droite
                    if point.x <= 10:
                        self.lin = 0.0
                        self.ang = -0.6
                    if point.x <= 40:
                        self.lin = 0.05
                        self.ang = -0.6
                    if point.x <= 70:
                        self.lin = 0.1
                        self.ang = -0.6
                elif point.y < 0 and point.y > -0.5 :   # Obstacles à gauche
                    if point.x <= 10:
                        self.lin = 0.0
                        self.ang = 0.6
                    if point.x <= 40:
                        self.lin = 0.05
                        self.ang = 0.6
                    if point.x <= 70:
                        self.lin = 0.1
                        self.ang = 0.6
                else:
                    self.lin = 0.1
                    self.ang = 0.0
        #else:
        #    self.lin = 1.0
            
        print(f"consigne : Lin = {self.lin} | Ang = {self.ang} | Old_lin = {self.old_lin}")
        '''
        if self.lin > self.old_lin:
            self.lin = 1.1*self.old_lin
        '''
        self.velo.linear.x = self.lin
        self.velo.angular.z = self.ang
        print(f"Lin : {self.lin} | Ang : {self.ang}")
        self.velocity_publisher.publish(self.velo)

        self.old_lin = self.lin
        

def reactive_move(args=None) :
    rclpy.init(args=args)
    autoRobot = AutoRobot()
    # Start the ros infinit loop with the move node.
    rclpy.spin(autoRobot)
    autoRobot.destroy_node()
    rclpy.shutdown()