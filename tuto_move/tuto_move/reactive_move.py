import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud

class AutoRobot(Node):
    def __init__(self):
        super().__init__('Auto')
        self.topic = '/multi/cmd_nav'
        # self.topic = '/cmd_vel'
        self.create_subscription( PointCloud, 'obstacles', self.avoid_obstacles, 10)
        self.velocity_publisher = self.create_publisher(Twist, self.topic, 10)
        self.velo = Twist()
        # self.timer = self.create_timer(0.1, self.avoid_obstacles)
        # self.lin = (float)(lin)
        # self.ang = (float)(ang)

    def avoid_obstacles(self, pntcld) :
        obstacles = pntcld.points
        sampleright1 = 0
        sampleright2 = 0
        sampleright3 = 0
        sampleleft1 = 0
        sampleleft2 = 0
        sampleleft3 = 0
        old_speed = self.velo

        for point in obstacles :
            if point.y >= 0 and point.y < 0.20 :
                if point.x >= 0 and point.x < 0.20:
                    sampleright1 += 1
                elif point.x >= 0.20 and point.x < 0.50 :
                    sampleright2 += 1
                elif point.x >= 0.50 :
                    sampleright3 += 1

            elif point.y < 0 and point.y > -0.20 :
                if point.x >= 0 and point.x < 0.20:
                    sampleleft1 += 1
                elif point.x >= 0.20 and point.x < 0.50 :
                    sampleleft2 += 1
                elif point.x >= 0.50 :
                    sampleleft3 += 1
        
        print('sampleleft1 : ' + (str)(sampleleft1) + ' sampleright1 :' + (str)(sampleright1))
        
        # if (sampleleft1 > 90 and sampleright1 > 90) :
        #     # self.velo.linear.x = 0.0
        #     # self.velo.angular.z = 0.8
        #     lin = 0.0
        #     ang = 0.6

        if (sampleleft1 > 30 or sampleright1 > 30) :

            if (sampleleft1 > sampleright1) : # Tourner à droite zone 1
                # self.velo.linear.x = 0.0
                # self.velo.angular.z = 0.6
                lin = 0.0
                ang = 0.6

            else : # Tourner à gauche zone 1
                # self.velo.linear.x = 0.0
                # self.velo.angular.z = -0.6
                lin = 0.0
                ang = -0.6


        elif (sampleleft2 > 30 or sampleright2 > 30) :

            if (sampleleft2 > sampleright2) : # Tourner à droite zone 2
                # self.velo.linear.x = 0.1
                # self.velo.angular.z = 0.6
                lin = 0.1
                ang = 0.6

            else : # Tourner à gauche zone 2
                # self.velo.linear.x = 0.1
                # self.velo.angular.z = -0.6
                lin = 0.1
                ang = -0.6
        
        elif (sampleleft3 > 30 or sampleright3 > 30) :

            if (sampleleft3 > sampleright3) : # Tourner à droite zone 3
                # self.velo.linear.x = 0.2
                # self.velo.angular.z = 0.6
                lin = 0.2
                ang = 0.6

            else : # Tourner à gauche zone 3
                # self.velo.linear.x = 0.2
                # self.velo.angular.z = -0.6
                lin = 0.2
                ang = -0.6
                
        else :
            # self.velo.linear.x = 0.4
            # self.velo.angular.z = 0.0
            lin = 0.4
            ang = 0.0
    
        if ((old_speed.linear.x < lin) and (ang != 0.6) and (ang != -0.6)) :
            if (old_speed.linear.x == 0) :
                lin = 0.05
            else :
                lin = old_speed.linear.x * 1.10 # Accélération linéaire de 10% par rapport à l'ancienne vitesse
        
        self.velo.linear.x = lin
        self.velo.angular.z = ang

        self.velocity_publisher.publish(self.velo)

def reactive_move(args=None) :
    rclpy.init(args=args)
    autoRobot = AutoRobot()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(autoRobot)
    autoRobot.destroy_node()
    rclpy.shutdown()