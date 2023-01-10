#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
import math
from geometry_msgs.msg import Point32

class ScanInterpret(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.obstacle_publish = self.create_publisher(PointCloud, 'obstacles', 10)

    def scan_callback(self, scanMsg):
        self.obstacles = []
        angle= scanMsg.angle_min
        for aDistance in scanMsg.ranges :
            if 0.05 < aDistance and aDistance < 0.6 :
                aPoint = Point32()
                aPoint.x= (float)(math.cos(angle) * aDistance)
                aPoint.y= (float)(math.sin( angle ) * aDistance)
                aPoint.z= (float)(0)
                self.obstacles.append( aPoint )
            angle+= scanMsg.angle_increment
        #sample= [ p for p in  self.obstacles[10:20] ]
        #self.get_logger().info( f" obs({len(self.obstacles)}) ...{sample}..." )
        #self.get_logger().info( f"scan:\n{scanMsg.header}\n{len(scanMsg.ranges)}" )

        obs = PointCloud()
        obs.header = scanMsg.header
        obs.points = self.obstacles
        self.obstacle_publish.publish(obs)

        

def main(args=None):
    rclpy.init(args=args)
    scanInterpret = ScanInterpret()
    rclpy.spin(scanInterpret)
    scanInterpret.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()