import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from spot_msgs.srv import SpotMotion

import sys, select, termios, tty


settings = termios.tcgetattr(sys.stdin)
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class myNode(Node):

    def __init__(self):
        super().__init__('my_Node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.key = ''
    

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.publisher_.publish(msg)
        self.i += 1
        
        self.key = getKey()
        if self.key == 's':
            self.call_service(False)
        

    def call_service(self, override):
        # create a request
        client = self.create_client(SpotMotion, '/Spot/sit_down')
        while not client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SpotMotion.Request()
        request.override = override

        # request to the server
        future = client.call_async(request)
        # received respone from the server
        return future.result()
        

def main(args=None):
    rclpy.init(args=args)

    my_node = myNode()
    rclpy.spin(my_node)
    
    #my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
