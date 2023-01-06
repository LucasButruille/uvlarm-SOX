import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveNode(Node):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz
        self.count = 0

    def activate(self):
        self.velo = Twist()
        self.velo.linear.x = 0.2 # target a 0.2 meter per second velocity
        self.velocity_publisher.publish(self.velo)
        self.Fini()

    def Fini(self):
        self.count += 1
        if self.count == 10/self.velo.linear.x:
            return True
        else:
            return False

    


def main(args=None):
    rclpy.init(args=args)
    move = MoveNode()

    while rclpy.ok() and not move.Fini:
        # Start the ros infinit loop with the move node.
        rclpy.spin_once(move)

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

if __name__ == '__main__':
    main()