import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud

class AutoRobot_Simu(Node):
    def __init__(self):
        super().__init__('Auto')
        self.topic = '/cmd_vel'
        #self.topic = '/multi/cmd_nav'
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
        sampletooleft1 = 0
        sampletooleft2 = 0
        sampletooleft3 = 0
        sampletooright1 = 0
        sampletooright2 = 0
        sampletooright3 = 0
        old_speed = self.velo

        for point in obstacles :
            if point.y >= 0 and point.y < 0.25 :
                if point.x >= 0 and point.x < 0.20:
                    sampleright1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampleright2 += 1
                elif point.x >= 0.60 :
                    sampleright3 += 1

            elif point.y < 0 and point.y > -0.25 :
                if point.x >= 0 and point.x < 0.20:
                    sampleleft1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampleleft2 += 1
                elif point.x >= 0.60 :
                    sampleleft3 += 1
            
            if point.y >= 0.25 and point.y < 0.40 :
                if point.x >= -0.05 and point.x < 0.20 :
                    sampletooright1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampletooright2 += 1
                elif point.x >= 0.60 :
                    sampletooright3 +=1

            elif point.y < 0.25 and point.y > -0.40 :
                if point.x >= -0.05 and point.x < 0.20 :
                    sampletooleft1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampletooleft2 += 1
                elif point.x >= 0.60 :
                    sampletooleft3 += 1
        
        # print('sampleleft1 : ' + (str)(sampleleft1) + ' sampleright1 :' + (str)(sampleright1))
        

        if (sampleleft1 > 5 or sampleright1 > 5) :

            if (sampleleft1 > sampleright1 and old_speed.angular.z >= 0 and sampletooright1 < 20) : # Tourner à droite zone 1
                # self.velo.linear.x = 0.0
                # self.velo.angular.z = 0.6
                lin = 0.0
                ang = 0.6

            elif (old_speed.angular.z <= 0 and sampletooleft1 < 20) : # Tourner à gauche zone 1
                # self.velo.linear.x = 0.0
                # self.velo.angular.z = -0.6
                lin = 0.0
                ang = -0.6
            
            else :
                lin = old_speed.linear.x
                ang = old_speed.angular.z


        elif (sampleleft2 > 5 or sampleright2 > 5) :

            if (sampleleft2 > sampleright2 and old_speed.angular.z >= 0.4 and sampletooright2 < 30) : # Tourner à droite zone 2
                # self.velo.linear.x = 0.1
                # self.velo.angular.z = 0.6
                lin = 0.1
                ang = 0.6

            elif (old_speed.angular.z <= 0 and sampletooleft2 < 30) : # Tourner à gauche zone 2
                # self.velo.linear.x = 0.1
                # self.velo.angular.z = -0.6
                lin = 0.1
                ang = -0.6

            else :
                lin = old_speed.linear.x
                ang = old_speed.angular.z
        
        elif (sampleleft3 > 5 or sampleright3 > 5) :

            if (sampleleft3 > sampleright3 and old_speed.angular.z >= 0 and sampletooright3 < 40) : # Tourner à droite zone 3
                # self.velo.linear.x = 0.2
                # self.velo.angular.z = 0.6
                lin = 0.2
                ang = 0.6

            elif (old_speed.angular.z <= 0 and sampletooleft3 < 40) : # Tourner à gauche zone 3
                # self.velo.linear.x = 0.2
                # self.velo.angular.z = -0.6
                lin = 0.2
                ang = -0.6

            else :
                lin = old_speed.linear.x
                ang = old_speed.angular.z
                
        else :
            # self.velo.linear.x = 0.4
            # self.velo.angular.z = 0.0
            lin = 0.4
            ang = 0.0
    
        if (old_speed.linear.x < lin) :
            if (old_speed.linear.x == 0) :
                lin = 0.05
            else :
                lin = old_speed.linear.x * 1.10 # Accélération linéaire de 10% par rapport à l'ancienne vitesse
        
        if (old_speed.linear.x > lin and lin != 0) :
            lin = old_speed.linear.x * 0.7 # Décélération linéaire de 30%
        
        self.velo.linear.x = lin
        self.velo.angular.z = ang

        self.velocity_publisher.publish(self.velo)
        

def reactive_move_simu(args=None) :
    rclpy.init(args=args)
    autoRobotsimu = AutoRobot_Simu()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(autoRobotsimu)
    autoRobotsimu.destroy_node()
    rclpy.shutdown()