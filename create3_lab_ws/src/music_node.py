#----------------------------------------
#
# @file     music_node.py
# @author   Jordan Reed
# @date     Nov 26, 2023
# @class    Robotics

# @desc     Node to play music
#
#
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import irobot_create_msgs
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration

from action_msgs.msg import GoalStatus


class Music(Node):
    def __init__(self, namespace:str):
        # call superclass init
        super().__init__('music')

        print(f"Constructing '{namespace}' music node.")
        self._namespace = namespace
        self.undock_sound_notes = [
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=250000000)),
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=250000000)),
            AudioNote(frequency=  0, max_runtime=Duration(sec=0, nanosec=120000000)),
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=250000000)),
            AudioNote(frequency=  0, max_runtime=Duration(sec=0, nanosec=120000000)),
            AudioNote(frequency=523, max_runtime=Duration(sec=0, nanosec=200000000)),
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=250000000)),
            AudioNote(frequency=  0, max_runtime=Duration(sec=0, nanosec=150000000)),
            AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec=300000000)),
            AudioNote(frequency=  0, max_runtime=Duration(sec=0, nanosec=300000000)),
            AudioNote(frequency=392, max_runtime=Duration(sec=0, nanosec=300000000))
        ]

        self.docking_sound_notes = [
            AudioNote(frequency=82, max_runtime=Duration(sec=0, nanosec=500000000)),
            AudioNote(frequency=87, max_runtime=Duration(sec=0, nanosec=250000000)),
            AudioNote(frequency= 0, max_runtime=Duration(sec=0, nanosec=250000000)),
            AudioNote(frequency=82, max_runtime=Duration(sec=0, nanosec=500000000)),
            AudioNote(frequency=87, max_runtime=Duration(sec=0, nanosec=250000000))
        ]

        self.docked_sound_notes = [
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=200000000)),
            AudioNote(frequency=466, max_runtime=Duration(sec=0, nanosec=200000000)),
            AudioNote(frequency=494, max_runtime=Duration(sec=0, nanosec=200000000)),
            AudioNote(frequency=523, max_runtime=Duration(sec=1, nanosec=0))
        ]
    
    def send_goal(self, action_type, action_name:str, goal):
        """Sets the action client and sends the goal to the robot. Spins node until result callback is received.

        :param action_type: Action type of the action to take
        :param str action_name: name of action to append to namespace command
        :param goal: goal object with necessary parameters to complete goal
        """

        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create/reuse action client with new goal info
        self._action_client = ActionClient(self, action_type, f'/{self._namespace}/{action_name}')

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
        while self.result == None and action_type != AudioNoteSequence:
            # rclpy.spin_once(self)
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
        
        try:
            # pos = DriveDistance.Result.pose
            self.pose = future.result().pose
            
            # print(f'current pose: ({self.pose.pose.position.x}, {self.pose.pose.position.y})')
        except Exception as e:
            # print(f"error getting pose: {e}")
            pass
    
    def send_music_goal(self, action):
        if action == 'undock':
            notes = self.undock_sound_notes
        elif action == 'docking':
            notes = self.docking_sound_notes
        else: # docked
            notes = self.docked_sound_notes

        temp_goal = AudioNoteSequence.Goal()
        note_sequence = AudioNoteVector()
        note_sequence.notes = notes
        temp_goal.iterations = 1
        temp_goal.note_sequence = note_sequence
        self.send_goal(AudioNoteSequence, 'audio_note_sequence', temp_goal)
        
