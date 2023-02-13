
import rclpy
import cv2 as cv
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from my_package.submodules.image_proc import imageProcessing as ip

class lane_f(Node):
    def __init__(self):
        super().__init__('my_lane_following')
        self.subscriber = self.create_subscription(Image, '/Spot/left_head_camera', self.image_callback, 1)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel',10)
        self.twist = Twist()
    
    def image_callback(self, imsg:Image):
        image = ip(imsg)
        M = image.find_centroid()
        
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv.circle(image.original, (cx, cy), 10, (255, 0, 0), -1)
            image.show_image()
            err = cx - 0.39*image.w
    
            self.twist.angular.z = -float(err)/130
            self.twist.linear.x = 0.7

            self.publisher_.publish(self.twist)
    
            

def main(args=None):
    rclpy.init(args = args)
    lane_node = lane_f()
    rclpy.spin(lane_node)
    lane_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
