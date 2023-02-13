import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class nav2(Node):
    def __init__(self):
        super().__init__('nav_node')
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.msg = PoseStamped()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = 'map'
        self.msg.pose.position.x = 6.0
        self.msg.pose.position.y = 8.0
        self.msg.pose.position.z = 0.0
        self.msg.pose.orientation.x = 0.0
        self.msg.pose.orientation.y = 0.0
        self.msg.pose.orientation.z = 0.0
        self.msg.pose.orientation.w = 1.0

    def goToPose(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.msg

        self._nav_client.wait_for_server()
        send_goal_future = self._nav_client.send_goal_async(goal_msg, feedback_callback=self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()


    def _feedbackCallback(self, feedback_msg):
        self._logger.info(str(feedback_msg))


def main(args=None):
    rclpy.init(args=args)
    nav = nav2()
    nav.goToPose()

    try:
        rclpy.spin(nav)
    except KeyboardInterrupt:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()