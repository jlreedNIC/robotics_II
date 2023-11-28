#----------------------------------------
#
# @file     main.py
# @author   Jordan Reed
# @date     Nov, 2023
# @class    Robotics

# @desc     This program will send goals to the Create3 robot, playing music and moving the robot.
#
# ------------------------------------------


import random
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from threading import RLock
from rclpy.executors import MultiThreadedExecutor
from key_commander import KeyCommander
from pynput.keyboard import KeyCode

from irobot_create_msgs.action import DriveDistance, Undock, Dock, RotateAngle, NavigateToPosition
from music_node import Music
from dock_node import Docker

from action_msgs.msg import GoalStatus

from irobot_create_msgs.srv import ResetPose
from geometry_msgs.msg import PoseStamped

# global var
rclpy.init()
namespace = "create3_05B9"
docker = Docker(namespace)
music_player = Music(namespace)
lock = RLock()

class MyNode(Node):
    """ A class that will create a single node, and reuse a single action client, to move a robot.
    """
    def __init__(self, namespace:str):
        """Initializes MyNode class with a given namespace. Has a single action client

        :param str namespace: what the namespace of robot is
        """
        # call superclass init
        super().__init__('walker')

        # 2 Seperate Callback Groups for handling the bumper Subscription and Action Clients
        self.cb_music_action = MutuallyExclusiveCallbackGroup()
        #cb_Action = cb_Subscripion
        self.cb_action =MutuallyExclusiveCallbackGroup()

        print(f"Constructing '{namespace}' node.")
        self._namespace = namespace
        self.result = None

        self._action_client = None

        # in order to reset pose
        self._reset_pose = self.create_client( ResetPose, f'/{self._namespace}/reset_pose')
        self.req = ResetPose.Request()

        self.home = PoseStamped()       # home location of robot
        self.pose = PoseStamped()
    
    def send_goal(self, action_type, action_name:str, goal, callback_group=None):
        """Sets the action client and sends the goal to the robot. Spins node until result callback is received.

        :param action_type: Action type of the action to take
        :param str action_name: name of action to append to namespace command
        :param goal: goal object with necessary parameters to complete goal
        """
        if callback_group == None:
            callback_group = self.cb_action

        # play music depending on the type of action
        if action_type == Undock:
            music_player.send_music_goal('undock')
        elif action_type == Dock:
            music_player.send_music_goal('docking')

        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create/reuse action client with new goal info
        self._action_client = ActionClient(self, action_type, f'/{self._namespace}/{action_name}', callback_group=callback_group)

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
            pass

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
        
        # get current position after goal done. used mainly for setting home position
        try:
            self.pose = future.result().pose
        except Exception as e:
            pass
    
    def drive(self):
        print('entered drive function')
        self._reset_pose.call(self.req) # reset pose

        # undock
        cur_goal = Undock.Goal()
        self.send_goal(Undock, 'undock', cur_goal)

        # drive for 1m
        cur_goal = DriveDistance.Goal()
        cur_goal.distance = 1.0
        self.send_goal(DriveDistance, 'drive_distance', cur_goal)

        # set home position
        self.home = self.pose
        self.home.pose.position.x -= .5

        # turn a random angle
        angle = random.uniform(-3.14, 3.14)
        cur_goal = RotateAngle.Goal()
        cur_goal.angle = float(angle)
        self.send_goal(RotateAngle, 'rotate_angle', cur_goal)

        # drive 0.5m
        cur_goal = DriveDistance.Goal()
        cur_goal.distance = 0.5
        self.send_goal(DriveDistance, 'drive_distance', cur_goal)

        # go back to home position
        goal = NavigateToPosition.Goal()
        goal.goal_pose = self.home
        self.send_goal(NavigateToPosition, 'navigate_to_position', goal)


        # dock robot
        # try:
        #     docker.dock()
        # except Exception as e:
        #     cur_goal = Dock.Goal()
        #     self.send_goal(Dock, 'dock', cur_goal)

        cur_goal = Dock.Goal()
        self.send_goal(Dock, 'dock', cur_goal)
        
        music_player.send_music_goal('docked')

def main():    
    roomba = MyNode(namespace)
    

    exec = MultiThreadedExecutor(4)
    exec.add_node(roomba)
    exec.add_node(docker)
    exec.add_node(music_player)

    keycom = KeyCommander([
        (KeyCode(char='a'), roomba.drive),
    ])
    print("'a' to start")
    
    try:
        exec.spin() # execute callbacks until shutdown or destroy is called
    except KeyboardInterrupt:
        print('KeyboardInterrupt, shutting down.')
        print("Shutting down executor")
        exec.shutdown()
        print("Destroying Nodes")
        roomba.destroy_node()
        docker.destroy_node()
        print("Shutting down RCLPY")
        rclpy.try_shutdown()

if __name__ == '__main__':
    print('--PYTHON SCRIPT--')
    main()