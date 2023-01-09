#!python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32

class ScanInterpret(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, scanMsg):
        self.get_logger().info( f"scan:\n{scanMsg.header} \n{len(scanMsg.ranges)}" )
        obstacles= []
        angle= scanMsg.angle_min
        for aDistance in scanMsg.ranges :
            if 0.05 < aDistance and aDistance < 0.3 :
                # aPoint= [
                #     math.cos(angle) * aDistance,
                #     math.sin( angle ) * aDistance
                # ]
                aPoint= Point32()
                aPoint.x= (float)(math.cos(angle) * aDistance)
                aPoint.y= (float)(math.sin( angle ) * aDistance)
                aPoint.z= (float)(0)
                obstacles.append( aPoint )
            angle+= scanMsg.angle_increment
        # sampleleft= [ p for p in  obstacles[0:(int)(len(obstacles)/2)] ]
        # self.get_logger().info( f" obs({len(sampleleft)}) ...{sampleleft}..." )
        # sampleright= [ p for p in  obstacles[(int)(len(obstacles)/2)+1:int(len(obstacles))] ]
        # self.get_logger().info( f" obs({len(sampleright)}) ...{sampleright}..." )

def main(args=None):
    rclpy.init(args=args)
    scanInterpret = ScanInterpret()
    rclpy.spin(scanInterpret)
    scanInterpret.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()