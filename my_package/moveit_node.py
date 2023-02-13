import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK

from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

import numpy as np
from scipy.spatial.transform import Rotation as R
from copy import deepcopy

target_angles = None
global_joint_states = None
tf_base_link_can = None


def skip_extra_rotation(j):
    k = []
    for f in j:
        if f > np.pi:
            f -= 2*np.pi
        if f < -np.pi:
            f += 2*np.pi
        k.append(f)
    return k


class MoveGroupActionClient(Node):
    def __init__(self, node:Node, target_angles):
        self.logger = node.get_logger()

        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = node.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True

        jc = JointConstraint()
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        j = skip_extra_rotation(target_angles)

        joints = {}
        joints['shoulder_pan_joint'] = j[0]
        joints['shoulder_lift_joint'] = j[1]
        joints['elbow_joint'] = j[2]
        joints['wrist_1_joint'] = j[3]
        joints['wrist_2_joint'] = j[4]
        # joints['wrist_3_joint'] = j[5]
        joints['wrist_3_joint'] = 0. # gripper always horizontal

        constraints = Constraints()
        for (joint, angle) in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.pipeline_id = 'move_group'
        self.motion_plan_request.group_name = 'spot_ur3e_arm'
        self.motion_plan_request.num_planning_attempts = 4
        self.motion_plan_request.allowed_planning_time = 4.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = True
        self.planning_options.look_around_attempts = 5
        self.planning_options.max_safe_execution_cost = 0.
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 4
        self.planning_options.replan_delay = 0.1

        self._action_client = ActionClient(node, MoveGroup, '/move_action')

    def send_goal(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info('Goal rejected :(')
            return

        self.logger.info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.logger.info(str(future))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.logger.info(str(feedback_msg))


class MinimalClientAsync():
    def __init__(self, node:Node):
        self.logger = node.get_logger()
        self.clock = node.get_clock()
        self.node = node

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.timer = node.create_timer(1.0, self.on_timer)
        node.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)

        self.cli = node.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')

    def on_timer(self):
        global tf_base_link_can
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'can5',
                Time())
            tf_base_link_can = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
            self.tf_base_link_can = tf_base_link_can
        except TransformException as ex:
            self.logger.info(
                f'Could not transform base_link to can: {ex}')
            return

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def moveit_ik(self, approach):
        group_name = 'spot_ur3e_arm'
        self.req.ik_request.group_name = group_name
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp = self.clock.now().to_msg()
        self.req.ik_request.pose_stamped.header.frame_id = 'base_link'

        rot_vert_base_link = 90 + np.rad2deg(np.arctan2(self.tf_base_link_can[1], self.tf_base_link_can[0])) * approach
        dist_can = 0.25

        xyz = [self.tf_base_link_can[0],self.tf_base_link_can[1],self.tf_base_link_can[2]]
        self.req.ik_request.pose_stamped.pose.position.x = xyz[0] - dist_can * np.sin(np.deg2rad(rot_vert_base_link))
        self.req.ik_request.pose_stamped.pose.position.y = xyz[1] + dist_can * np.cos(np.deg2rad(rot_vert_base_link))
        self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.03

        abc = [180, 0, rot_vert_base_link]
        r = R.from_euler('xyz',[np.deg2rad(a) for a in abc])
        self.req.ik_request.pose_stamped.pose.orientation.x = r.as_quat()[0]
        self.req.ik_request.pose_stamped.pose.orientation.y = r.as_quat()[1]
        self.req.ik_request.pose_stamped.pose.orientation.z = r.as_quat()[2]
        self.req.ik_request.pose_stamped.pose.orientation.w = r.as_quat()[3]

        self.req.ik_request.timeout.sec = 5
        print(self.req.ik_request)
        return self.req.ik_request

    def send_request(self, approach):
        self.req = GetPositionIK.Request()
        self.req.ik_request = self.moveit_ik(approach)

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    


def main():
    rclpy.init()
    node = Node('moveit_ik_python')

    minimal_client = MinimalClientAsync(node)
    while global_joint_states is None or tf_base_link_can is None:
        rclpy.spin_once(node)
    response = minimal_client.send_request(approach=1)
    target_angles = list(response.solution.joint_state.position)[:6]

    if not len(target_angles):
        response = minimal_client.send_request(approach=-1)
        target_angles = list(response.solution.joint_state.position)[:6]

    if not len(target_angles):
        print('No IK solution found')
        node.destroy_node()
        rclpy.shutdown()
        return
    print([round(np.rad2deg(ta)) for ta in target_angles])

    action_client = MoveGroupActionClient(node, target_angles)
    action_client.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

