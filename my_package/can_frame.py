import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

import numpy as np
from scipy.spatial.transform import Rotation as R


class FrameListener(Node):
    def __init__(self):
        super().__init__('can_listener')
        self.target_frame = self.declare_parameter('target_frame', 'base_link').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.subscriber = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.subscriber

        self.look_transform()
        self.call_service()


    def look_transform(self):
        try:
            t= self.tf_buffer.lookup_transform(self.target_frame, 'can1', Time(), Duration(seconds=3.0))
            self.tf_base_link_can = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            
        except:
            self.get_logger().info('cannot transform')

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def call_service(self):
        client = self.create_client(GetPositionIK,'/compute_ik')
    
        while not client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')

        #create a request, request to server, receive response from server
        request = self.request()
        self.future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)
        respone = self.future.result()
        print(respone)

    
    def listener_callback(self, msg:JointState):
        self.joint_state = msg

    
    def request(self):
        request = GetPositionIK.Request()

        request.ik_request.group_name = 'spot_ur3e_arm'
        request.ik_request.timeout.sec = 5

        request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.avoid_collisions = True

        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()

        rot_vert_base_link = 90 + np.rad2deg(np.arctan2(self.tf_base_link_can[1], self.tf_base_link_can[0])) * (-1)
        dist_can = 0.1

        # translation in 3 dimesions x,y,z
        xyz = [self.tf_base_link_can[0],self.tf_base_link_can[1], self.tf_base_link_can[2]]        
        request.ik_request.pose_stamped.pose.position.x = xyz[0] - dist_can * np.sin(np.deg2rad(rot_vert_base_link))
        request.ik_request.pose_stamped.pose.position.y = xyz[1] - dist_can * np.cos(np.deg2rad(rot_vert_base_link))
        request.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.03

        # rotation about z axis "roll"
        abc = [180, 0, rot_vert_base_link]
        r = R.from_euler('xyz', [np.deg2rad(a) for a in abc])
        request.ik_request.pose_stamped.pose.orientation.x = r.as_quat()[0]
        request.ik_request.pose_stamped.pose.orientation.y = r.as_quat()[1]
        request.ik_request.pose_stamped.pose.orientation.z = r.as_quat()[2]
        request.ik_request.pose_stamped.pose.orientation.w = r.as_quat()[3]

        #print(request)
        return request




def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
