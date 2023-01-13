import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud

class AutoRobot(Node):
    def __init__(self):
        super().__init__('Auto')
        #self.topic = '/cmd_vel'
        self.topic = '/multi/cmd_nav'
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
        sampleright1 = 0
        sampleright2 = 0
        sampleright3 = 0
        sampleleft1 = 0
        sampleleft2 = 0
        sampleleft3 = 0

        for point in obstacles :
            if point.y >= 0 and point.y < 0.25 :
                if point.x >= 0 and point.x < 0.15:
                    sampleright1 += 1
                elif point.x >= 0.15 and point.x < 0.30 :
                    sampleright2 += 1
                elif point.x >= 0.30 :
                    sampleright3 += 1

            elif point.y < 0 and point.y > -0.25 :
                if point.x >= 0 and point.x < 0.15:
                    sampleleft1 += 1
                elif point.x >= 0.15 and point.x < 0.30 :
                    sampleleft2 += 1
                elif point.x >= 0.30 :
                    sampleleft3 += 1
        
        print('sampleleft1 : ' + (str)(sampleleft1) + ' sampleright1 :' + (str)(sampleright1))
        print('sampleleft2 : ' + (str)(sampleleft2) + ' sampleright1 :' + (str)(sampleright2))
        print('sampleleft3 : ' + (str)(sampleleft3) + ' sampleright1 :' + (str)(sampleright3))
        print("\n")

        if (sampleleft1 > 150 and sampleright1 > 150):
            self.velo.linear.x = 0.0
            self.velo.angular.Z = 0.8
        
        elif (sampleleft1 > 30 or sampleright1 > 30) :            
            if (sampleleft1 > sampleright1) : # Tourner à droite zone 1
                self.velo.linear.x = 0.0
                self.velo.angular.z = 0.6
            else : # Tourner à gauche zone 1
                self.velo.linear.x = 0.0
                self.velo.angular.z = -0.6

        elif (sampleleft2 > 30 or sampleright2 > 30) :
            if (sampleleft2 > sampleright2) : # Tourner à droite zone 2
                self.velo.linear.x = 0.1
                self.velo.angular.z = 0.6
            else : # Tourner à gauche zone 2
                self.velo.linear.x = 0.1
                self.velo.angular.z = -0.6
        
        elif (sampleleft3 > 30 or sampleright3 > 30) :
            if (sampleleft3 > sampleright3) : # Tourner à droite zone 3
                self.velo.linear.x = 0.2
                self.velo.angular.z = 0.6
            else : # Tourner à gauche zone 3
                self.velo.linear.x = 0.2
                self.velo.angular.z = -0.6
                
        else :
            self.velo.linear.x = 0.4
            self.velo.angular.z = 0.0
    
        self.velocity_publisher.publish(self.velo)

        self.old_lin = self.lin
        

def reactive_move(args=None) :
    rclpy.init(args=args)
    autoRobot = AutoRobot()
    # Start the ros infinit loop with the move node.
    rclpy.spin(autoRobot)
    autoRobot.destroy_node()
    rclpy.shutdown()