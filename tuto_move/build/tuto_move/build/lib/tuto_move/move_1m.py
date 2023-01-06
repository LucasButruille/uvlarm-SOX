import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveNode(Node):

    def __init__(self):
        super().__init__('move')
        self.topic = '/turtle1/cmd_vel'
        #self.topic = '/multi/cmd_nav'
        self.velocity_publisher = self.create_publisher(Twist, self.topic, 10)
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz

    def activate(self):
        self.velo = Twist()
        self.velo.linear.x = 0.2 # target a 0.2 meter per second velocity
        self.velocity_publisher.publish(self.velo)
    

def main(args=None):
    rclpy.init(args=args)
    move = MoveNode()

    count = 0
    fini = False
    # Start the ros infinit loop with the move node.
    # rclpy.spin(move)
    while rclpy.ok() and not fini:
        rclpy.spin_once(move)
        temps = 10/move.velo.linear.x
        if count >= temps:
            fini = True
        else:
            fini = False
        count += 1


    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

if __name__ == '__main__':
    main()