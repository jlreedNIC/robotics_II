#-------------------------------------------
#
# @file     ros2_node.py
# @author   Jordan Reed
# @date     November, 2023
# @brief    Creates a basic ros2 single threaded node based on api by James Lasso
#
# ------------------------------------------

import rclpy                                    # only used when single threaded
from rclpy.node import Node
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus

# fanuc specific code
import sys
sys.path.append("larry_ros2_demo/fanuc_ros2_drivers")
# from fanuc_ros2_drivers.fanuc_ros2_interfaces.action import WriteJointOffset, WriteJointPose, WriteJointPosition
from fanuc_ros2_interfaces.action import WriteJointOffset, WriteJointPose, WriteJointPosition


class MyNode(Node):
    """ A class that will create a single basic node, and reuse a single action client, to move a robot.
    """
    def __init__(self):
        """Initializes MyNode class Has a single action client

        :param str namespace: what the namespace of robot is
        """
        # call superclass init
        super().__init__('ros2_test')

        self.result = None

        self._action_client = None

    def send_joint_pose_goal(self, joints:list):
        """ Function to pass a list of joints into Goal object and send to action server for using Write Joint Pose action.

        :param list joints: array of floats for joint positions
        """
        if len(joints) != 6:
            print(f'6 joint positions needed. {len(joints)} passed.')
            exit(1)
        
        goal = WriteJointPose.Goal()
        goal.joint1 = float(joints[0])
        goal.joint2 = float(joints[1])
        goal.joint3 = float(joints[2])
        goal.joint4 = float(joints[3])
        goal.joint5 = float(joints[4])
        goal.joint6 = float(joints[5])
        self.send_goal(WriteJointPose, "WriteJointPose", goal)

    def send_joint_offset_goal(self, joint:int, value):
        """Function to more easily send a goal of write joint offset

        :param int joint: which joint to affect
        :param _type_ value: how much to offset joint by, should be float
        """
        goal = WriteJointOffset.Goal()
        goal.joint = joint
        goal.value = float(value)
        self.send_goal(WriteJointOffset, "WriteJointOffset", goal)
    
    def send_joint_position_goal(self, joint:int, value):
        """Function to more easily send a goal of write joint offset

        :param int joint: which joint to affect
        :param _type_ value: how much to offset joint by, should be float
        """
        goal = WriteJointPosition.Goal()
        goal.joint = joint
        goal.value = float(value)
        self.send_goal(WriteJointPosition, "WriteJointPosition", goal)

    def send_goal(self, action_type, action_name:str, goal):
        """Sets the action client and sends the goal to the robot. Spins node until result callback is received.

        :param action_type: Action type of the action to take
        :param str action_name: name of action to append to namespace command
        :param goal: goal object with necessary parameters to complete goal
        """
        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create/reuse action client with new goal info
        self._action_client = ActionClient(self, action_type, f'/{action_name}')

        # wait for server
        self.get_logger().warning("Waiting for server...")
        self._action_client.wait_for_server()

        # server available
        self.get_logger().warning("Server available. Sending goal now...")

        # send goal
        self.send_goal_future = self._action_client.send_goal_async(goal) #, self.feedback_callback)

        # add done callback for response
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # spin node until done
        self.result = None
        while self.result == None:
            rclpy.spin_once(self)   # only used when single threaded

        # goal done
        self.get_logger().warning(f"{action_name} action done")
    
    def feedback_callback(self, feedback):
        """Callback when getting feedback. Will print feedback

        :param _type_ feedback: _description_
        """
        self.get_logger().info(f'received feedback: {feedback}')
    
    def goal_response_callback(self, future):
        """Callback when a response has been received. Calls get result callback.

        :param _type_ future: _description_
        """
        goal_handle = future.result()
        # print(f'goal handle: {goal_handle}')
        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED")
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Prints result of goal. Sets self.result variable in order to stop spinning node.

        :param _type_ future: _description_
        """
        self.result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Succeeded! Result info hidden.")
        else:
            self.get_logger().error(f"Goal Failed with status: {status}")
