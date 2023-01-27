import rclpy
import sys
sys.path.append('/install/kobuki_ros_interfaces/lib/python.3.8/site-packages')
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from kobuki_ros_interfaces.msg import WheelDropEvent
from kobuki_ros_interfaces.msg import BumperEvent
from kobuki_ros_interfaces.msg import ButtonEvent
from kobuki_ros_interfaces.msg import Led
from kobuki_ros_interfaces.msg import Sound
from std_msgs.msg import Bool, Float64, Int64
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseStamped


class AutoRobot(Node):
    def __init__(self):
        super().__init__('Auto')
        self.topic = '/multi/cmd_nav'
        # self.topic = '/cmd_vel'
        self.create_subscription(PointCloud, 'obstacles', self.laser_data, 10)
        self.velocity_publisher = self.create_publisher(Twist, self.topic, 10)
        self.velo = Twist()

        self.create_subscription(WheelDropEvent, '/events/wheel_drop', self.WheelDrop, 10)

        self.create_subscription(ButtonEvent, '/events/button', self.Button, 10)
        self.led1_publisher = self.create_publisher(Led, '/commands/led1', 10)
        self.led2_publisher = self.create_publisher(Led, '/commands/led2', 10)

        self.led1 = Led()
        self.led2 = Led()

        self.led1.value = 3
        self.led2.value = 3

        self.led1_publisher.publish(self.led1)
        self.led2_publisher.publish(self.led2)

        self.sound_publisher = self.create_publisher(Sound, '/commands/sound', 10)
        self.melodie = Sound()
        
        self.create_subscription(BumperEvent, '/events/bumper', self.Bumper, 10)
        self.b0 = False
        self.b1 = False
        self.b2 = True

        self.create_subscription(Twist, '/slam_nav', self.Maps, 10)

        self.timer = self.create_timer(0.05, self.Mouvement)

        self.create_subscription(Bool, '/detection_objet', self.Detection_objet, 10)

        self.create_subscription(Float64, '/distance', self.Distance, 10)

        self.create_subscription(Int64, '/bottle_position', self.Bottle_position, 10)

        self.create_subscription(MarkerArray, '/visualization_marker', self.Marker_bottles, 10)

        self.bottleOKpub = self.create_publisher(Bool, '/bottleOK', 10)
        
        self.create_subscription(PoseStamped, '/robot_pose', self.Robot_map, 10)

        self.lin = (float)
        self.ang = (float)
        self.lin2 = 0.0
        self.ang2 = 0.0
        self.detect_tab = []
        self.objet = -1
        self.robot_prox = False
        self.count_temp = 0
        self.bottleOK = Bool()
        self.distance_bottle = 0
        self.marqueur = MarkerArray()


    def WheelDrop(self, drop_event) :
        if drop_event.state == 1 :
            self.b0 = False
            self.b1 = False
            self.b2 = True
            self.led1.value = 3
            self.led1_publisher.publish(self.led1)
            self.led2.value = 3
            self.led2_publisher.publish(self.led2)
            self.melodie.value = 2
            self.sound_publisher.publish(self.melodie)
            

    def Bumper(self, bump_event) :
        if bump_event.state == 1 :
            self.b0 = False
            self.b1 = False
            self.b2 = True
            self.led1.value = 3
            self.led1_publisher.publish(self.led1)
            self.led2.value = 3
            self.led2_publisher.publish(self.led2)
            self.melodie.value = 2
            self.sound_publisher.publish(self.melodie)

    def Button(self, button_event) :
        if button_event.button == 0 and button_event.state == 0 :
            self.b0 = True
            self.b1 = False
            self.b2 = False
            self.led1.value = 1
            self.led1_publisher.publish(self.led1)
            self.led2.value = 3
            self.led2_publisher.publish(self.led2)
            self.melodie.value = 0
            self.sound_publisher.publish(self.melodie)

        elif button_event.button == 1 and button_event.state == 0 :
            self.b0 = False
            self.b1 = True
            self.b2 = False
            self.led1.value = 3
            self.led1_publisher.publish(self.led1)
            self.led2.value = 1
            self.led2_publisher.publish(self.led2)
            self.melodie.value = 0
            self.sound_publisher.publish(self.melodie)

        elif button_event.button == 2 and button_event.state == 0 :
            self.b0 = False
            self.b1 = False
            self.b2 = True
            self.led1.value = 0
            self.led1_publisher.publish(self.led1)
            self.led2.value = 0
            self.led2_publisher.publish(self.led2)
            self.melodie.value = 1
            self.sound_publisher.publish(self.melodie)

    def Maps(self, road) :
        self.lin2 = road.linear.x
        self.ang2 = road.angular.z

    def laser_data(self, points) :
        self.pntcld = points

    def Mouvement(self) :
        if self.b0 == True :
            self.Avoid_obstacles()
        elif self.b1 == True :
            self.SLAM()
        elif self.b2 == True :
            self.velo.linear.x = 0.0
            self.velo.angular.z = 0.0
            # self.velocity_publisher.publish(self.velo)
        
        # print("b0 : " + str(self.b0) + "b1 : " + str(self.b1) + "b2 : " + str(self.b2))

    def Detection_objet(self, detect) :
        if len(self.detect_tab) < 30 :
            if detect.data == False :
                self.detect_tab.append(-1)
            else :
                self.detect_tab.append(+1)

        else :
            count = 0
            for i in self.detect_tab :
                count += self.detect_tab[i]

            self.detect_tab = []
            
            if count > 25 :
                self.objet = +1
            else :
                self.objet = -1



    def Distance(self, dist) :
        self.distance_bottle = dist.data
        if dist.data < 0.6 and dist.data > 0.0:
            self.distance = -1

        elif dist.data > 0.7 :
            self.distance = +1

        elif dist.data == 0.0 :
            self.distance = +2

        else :
            self.distance = 0

    def Bottle_position(self, pos):
        if pos.data < 424 - 100 :
            self.position = +1
        elif pos.data > 424 + 100 :
            self.position = -1
        else :
            self.position = 0

    def Robot_map(self, robot_pose) :
        self.robot_x = robot_pose.pose.position.x
        self.robot_y = robot_pose.pose.position.y
        self.robot_prox = False
        for i in range(len(self.marqueur.markers)) :
            if self.robot_x > (self.marqueur.markers[i].pose.position.x - 0.7) and self.robot_x < (self.marqueur.markers[i].pose.position.x + 0.7) and self.robot_y > (self.marqueur.markers[i].pose.position.y - 0.7) and self.robot_y < (self.marqueur.markers[i].pose.position.y + 0.7) :
                self.robot_prox = True
        print(self.robot_prox)
        

    def Marker_bottles(self, mark) :
        self.marqueur = mark


    def Avoid_obstacles(self) :

        obstacles = self.pntcld.points
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

        if self.objet == -1 or self.distance == +2 or self.robot_prox == True :

            self.count_temp = 0

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
                    self.lin = 0.0
                    self.ang = -0.8

                elif (sampleright1 > sampleleft1 and old_speed.angular.z >= 0 and sampletooleft1 < 10) : # Tourner à gauche zone 1
                    self.lin = 0.0
                    self.ang = 0.8
                
                else :
                    if old_speed.angular.z <= 0:
                        self.lin = 0.0
                        self.ang = -0.8
                    else:
                        self.lin = 0.0
                        self.ang = 0.8

            # Zone 2
            elif (sampleleft2 > 10 or sampleright2 > 10) :

                if (sampleleft2 > sampleright2 and old_speed.angular.z <= 0 and sampletooright2 < 10) : # Tourner à droite zone 2
                    self.lin = 0.2
                    self.ang = -0.6

                elif (sampleright2 > sampleleft2 and old_speed.angular.z >= 0 and sampletooleft2 < 10) : # Tourner à gauche zone 2
                    self.lin = 0.2
                    self.ang = 0.6

                else :
                    if old_speed.angular.z <= 0:
                        self.lin = 0.2
                        self.ang = -0.6
                    else:
                        self.lin = 0.2
                        self.ang = 0.6
            
            # Zone 3
            elif (sampleleft3 > 10 or sampleright3 > 10) :

                if (sampleleft3 > sampleright3 and old_speed.angular.z <= 0 and sampletooright3 < 10) : # Tourner à droite zone 3
                    self.lin = 0.3
                    self.ang = -0.6

                elif (sampleright3 > sampleleft3 and old_speed.angular.z >= 0 and sampletooleft3 < 10) : # Tourner à gauche zone 3
                    self.lin = 0.3
                    self.ang = 0.6

                else :
                    if old_speed.angular.z <= 0:
                        self.lin = 0.3
                        self.ang = -0.6
                    else:
                        self.lin = 0.3
                        self.ang = 0.6

            # Zones éloignées sur les cotés 
            elif (sampletooleft1 > 5 or sampletooright1 > 5):
                if (sampletooleft1 > sampletooright1) :         # Tourner légerement à droite
                    self.lin = 0.4
                    self.ang = -0.8
                else :                                          # Tourner légerement à gauche
                    self.lin = 0.4
                    self.ang = 0.8

            elif (sampletooleft2 > 5 or sampletooright2 > 5) :
                if (sampletooleft2 > sampletooright2) :         # Tourner légerement à droite
                    self.lin = 0.4
                    self.ang = -0.6
                else :                                          # Tourner légerement à gauche
                    self.lin = 0.4
                    self.ang = 0.6

            elif (sampletooleft3 > 5 or sampletooright3 > 5) :
                if (sampletooleft3 > sampletooright3) :         # Tourner légerement à droite
                    self.lin = 0.4
                    self.ang = -0.4
                else :                                          # Tourner légerement à gauche
                    self.lin = 0.4
                    self.ang = 0.4

            else :
                self.lin = 0.4
                self.ang = 0.0

        else :
            if self.position == -1 :
                self.lin = 0.0
                self.ang = -0.4

            elif self.position == +1 :
                self.lin = 0.0
                self.ang = 0.4

            else :
                if self.distance == -1 :
                    self.lin = -0.1
                    self.ang = 0.0
                elif self.distance == +1 :
                    self.lin = 0.1
                    self.ang = 0.0
                else :
                    self.lin = 0.0
                    self.ang = 0.0
                    self.count_temp += 1
                    if self.count_temp >= 60 :
                        self.count_temp = 0
                        self.bottleOK.data = True
                        self.bottleOKpub.publish(self.bottleOK)
                        self.melodie.value = 1
                        self.sound_publisher.publish(self.melodie)
        
        # Accélération
        if (old_speed.linear.x < self.lin and self.lin >= 0) : # Linéaire de 10%
            if (old_speed.linear.x <= 0) :
                self.lin = 0.05
            else :
                self.lin = min(self.lin, old_speed.linear.x * 1.10)
        
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
        
        # print('lin : ' + str(lin) + ' | ang : ' + str(ang))

        self.velo.linear.x = self.lin
        self.velo.angular.z = self.ang
        self.velocity_publisher.publish(self.velo)

    def SLAM(self) :
        self.velo.linear.x = 1.5 * self.lin2
        self.velo.angular.z = 1.5 * self.ang2
        self.velocity_publisher.publish(self.velo)
        

def reactive_move(args=None) :
    rclpy.init(args=args)
    autoRobot = AutoRobot()
    # Start the ros infinit loop with the AutoRobot node.
    rclpy.spin(autoRobot)
    autoRobot.destroy_node()
    rclpy.shutdown()