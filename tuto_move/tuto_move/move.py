import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveNode(Node):

    def __init__(self, lin=0.0, ang=0.0):
        super().__init__('move')
        #self.topic = '/turtle1/cmd_vel'
        self.topic = '/cmd_vel'
        #self.topic = '/multi/cmd_nav'
        self.velocity_publisher = self.create_publisher(Twist, self.topic, 10)
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz
        self.lin = (float)(lin)
        self.ang = (float)(ang)

    def activate(self):
        self.velo = Twist()
        self.velo.linear.x = self.lin # target a 0.2 meter per second velocity
        self.velo.angular.z = self.ang 
        self.velocity_publisher.publish(self.velo)
    

def move_1m(args=None):
    rclpy.init(args=args)
    move = MoveNode(0.2)

    count = 0
    fini = False
    # Start the ros infinit loop with the move node.
    # rclpy.spin(move)
    while rclpy.ok() and not fini:
        rclpy.spin_once(move)
        temps = 10/abs(move.velo.linear.x)
        if count >= temps:
            fini = True
        else:
            fini = False
        count += 1


    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

def rear(args=None):
    rclpy.init(args=args)
    move = MoveNode(lin=-0.2)

    count = 0
    fini = False
    # Start the ros infinit loop with the move node.
    # rclpy.spin(move)
    while rclpy.ok() and not fini:
        rclpy.spin_once(move)
        temps = 10/abs(move.velo.linear.x)
        if count >= temps:
            fini = True
        else:
            fini = False
        count += 1


    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

def turn_left(args=None):
    rclpy.init(args=args)
    turn_left = MoveNode(ang=1.0)

    count = 0
    fini = False
    # Start the ros infinit loop with the move node.
    # rclpy.spin(move)
    while rclpy.ok() and not fini:
        rclpy.spin_once(turn_left)
        temps = 10/abs(turn_left.velo.angular.z)
        if count >= temps:
            fini = True
        else:
            fini = False
        count += 1


    # At the end, destroy the node explicitly.
    turn_left.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

def turn_right(args=None):
    rclpy.init(args=args)
    turn_left = MoveNode(ang=-1.0)

    count = 0
    fini = False
    # Start the ros infinit loop with the move node.
    # rclpy.spin(move)
    while rclpy.ok() and not fini:
        rclpy.spin_once(turn_left)
        temps = 10/abs(turn_left.velo.angular.z)
        if count >= temps:
            fini = True
        else:
            fini = False
        count += 1


    # At the end, destroy the node explicitly.
    turn_left.destroy_node()

    # and shut the light down.
    rclpy.shutdown()
 