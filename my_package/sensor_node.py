import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class sensorNode(Node):
    def __init__(self):
        super().__init__("sensor_Node")
        self.subscribe_ = self.create_subscription(Range, '/Spot/distance_sensor', self.listener_callback, 10)
        self.subscribe_
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
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
        self.publisher_.publish(msg)
        self.i += 1
       

    def listener_callback(self, msg:Range):
        if msg.range < 2:
            self.too_close = True
           


def main(args=None):
    rclpy.init(args = args)
    sensor_node = sensorNode()

    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
