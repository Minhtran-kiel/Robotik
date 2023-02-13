import rclpy
from math import radians
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class scannerNode(Node):
    def __init__(self):
        super().__init__("sensor_Node")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_twist = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscribtion_scan = self.create_subscription(LaserScan, '/scan', self.listener_callback, qos_profile=qos_profile)
        self.subscribtion_scan
       
        self.publisher_sensor = self.create_publisher(LaserScan, '/cone_scan', qos_profile=qos_profile)
        self.subscribtion = self.create_subscription(LaserScan, '/cone_scan', self.listener_callback_2, qos_profile=qos_profile)
        self.subscribtion
        
    
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.too_close = False

    def timer_callback(self):
        msg = Twist()

        if self.too_close == True:
            msg.linear.x = 0.0
        else:
            msg.linear.x = 1.0
        self.publisher_twist.publish(msg)
        self.i += 1

    def listener_callback(self, msg:LaserScan):
        msg.angle_max = radians(20)
        msg.angle_min = radians(-20)
        self.publisher_sensor.publish(msg)
       

    def listener_callback_2(self, msg:LaserScan):
        #print(len(msg.ranges))
        if msg.ranges[361] < 2:
            self.too_close = True
           


def main(args=None):
    rclpy.init(args = args)
    scaner_node = scannerNode()

    rclpy.spin(scaner_node)
    scaner_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
    
