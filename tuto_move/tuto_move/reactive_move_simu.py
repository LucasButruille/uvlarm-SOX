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
            if point.y >= 0 and point.y < 0.20 :
                if point.x >= 0 and point.x < 0.20:
                    sampleleft1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampleleft2 += 1
                elif point.x >= 0.60 :
                    sampleleft3 += 1

            elif point.y < 0 and point.y > -0.20 :
                if point.x >= 0 and point.x < 0.20:
                    sampleright1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampleright2 += 1
                elif point.x >= 0.60 :
                    sampleright3 += 1
            
            if point.y >= 0.20 and point.y < 0.30 :
                if point.x >= -0.05 and point.x < 0.20 :
                    sampletooleft1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampletooleft2 += 1
                elif point.x >= 0.60 :
                    sampletooleft3 +=1

            elif point.y < -0.20 and point.y > -0.30 :
                if point.x >= -0.05 and point.x < 0.20 :
                    sampletooright1 += 1
                elif point.x >= 0.20 and point.x < 0.60 :
                    sampletooright2 += 1
                elif point.x >= 0.60 :
                    sampletooright3 += 1
        
        # print(" ")
        # print('sampletooleft1 : ' + (str)(sampletooleft1) + ' | ' + ' sampleleft1 :' + (str)(sampleleft1) + ' | ' + ' sampleright1 :' + (str)(sampleright1) + ' | ' + ' sampletooright1 :' + (str)(sampletooright1))
        # print('sampletooleft2 : ' + (str)(sampletooleft2) + ' | ' + ' sampleleft2 :' + (str)(sampleleft2) + ' | ' + ' sampleright2 :' + (str)(sampleright2) + ' | ' + ' sampletooright2 :' + (str)(sampletooright2))
        # print('sampletooleft3 : ' + (str)(sampletooleft3) + ' | ' + ' sampleleft3 :' + (str)(sampleleft3) + ' | ' + ' sampleright3 :' + (str)(sampleright3) + ' | ' + ' sampletooright3 :' + (str)(sampletooright3))
        
        # Zone 1
        if (sampleleft1 > 10 or sampleright1 > 10) : # Si plus de 10 points dans la zone 1, gauche ou droite

            if (sampleleft1 > sampleright1 and old_speed.angular.z <= 0 and sampletooright1 < 10) : # Tourner à droite zone 1
                lin = 0.0
                ang = -1.0

            elif (sampleright1 > sampleleft1 and old_speed.angular.z >= 0 and sampletooleft1 < 10) : # Tourner à gauche zone 1
                lin = 0.0
                ang = 1.0
            
            else :
                if old_speed.angular.z <= 0:
                    lin = 0.0
                    ang = -1.0
                else:
                    lin = 0.0
                    ang = 1.0

        # Zone 2
        elif (sampleleft2 > 10 or sampleright2 > 10) :

            if (sampleleft2 > sampleright2 and old_speed.angular.z <= 0 and sampletooright2 < 10) : # Tourner à droite zone 2
                lin = 0.1
                ang = -0.6

            elif (sampleright2 > sampleleft2 and old_speed.angular.z >= 0 and sampletooleft2 < 10) : # Tourner à gauche zone 2
                lin = 0.1
                ang = 0.6

            else :
                if old_speed.angular.z <= 0:
                    lin = 0.1
                    ang = -0.6
                else:
                    lin = 0.1
                    ang = 0.6
        
        # Zone 3
        elif (sampleleft3 > 10 or sampleright3 > 10) :

            if (sampleleft3 > sampleright3 and old_speed.angular.z <= 0 and sampletooright3 < 10) : # Tourner à droite zone 3
                lin = 0.4
                ang = -0.6

            elif (sampleright3 > sampleleft3 and old_speed.angular.z >= 0 and sampletooleft3 < 10) : # Tourner à gauche zone 3
                lin = 0.4
                ang = 0.6

            else :
                if old_speed.angular.z <= 0:
                    lin = 0.4
                    ang = -0.6
                else:
                    lin = 0.4
                    ang = 0.6

        # Zones éloignées sur les cotés 
        elif (sampletooleft1 > 5 or sampletooright1 > 5):
            if (sampletooleft1 > sampletooright1) :         # Tourner légerement à droite
                lin = 0.4
                ang = -0.6
            else :                                          # Tourner légerement à gauche
                lin = 0.4
                ang = 0.6

        elif (sampletooleft2 > 5 or sampletooright2 > 5) :
            if (sampletooleft2 > sampletooright2) :         # Tourner légerement à droite
                lin = 0.4
                ang = -0.4
            else :                                          # Tourner légerement à gauche
                lin = 0.4
                ang = 0.4

        elif (sampletooleft3 > 5 or sampletooright3 > 5) :
            if (sampletooleft3 > sampletooright3) :         # Tourner légerement à droite
                lin = 0.4
                ang = -0.3
            else :                                          # Tourner légerement à gauche
                lin = 0.4
                ang = 0.3

        else :
            lin = 0.4
            ang = 0.0

        
        # Accélération
        if (old_speed.linear.x < lin) : # Linéaire de 10%
            if (old_speed.linear.x == 0) :
                lin = 0.05
            else :
                lin = min(lin, old_speed.linear.x * 1.10)
        
        # elif (old_speed.angular.z < ang) : # Angulaire de 20%
        #     if (old_speed.angular.z == 0) :
        #         ang = 0.05
        #     else :
        #         ang = min(ang, old_speed.angular.z * 1.20)

        # # Décélération
        # elif (old_speed.linear.x > lin and lin != 0) : # Linéaire de 30%
        #     lin = max(lin, old_speed.linear.x * 0.7)
        
        # elif (old_speed.angular.z > ang and ang != 0) : # Angulaire de 30%
        #     ang = max(ang, old_speed.angular.z * 0.7)
        
        # else:
        #     print("Vitesse maintenue")
        
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