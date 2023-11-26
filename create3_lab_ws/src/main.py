#----------------------------------------
#
# @file     main.py
# @author   Jordan Reed
# @date     Jan 30, 2023
# @class    Robotics

# @desc     This program will send goals to the Create3 robot so the 
#           robot will move forward 1m, turn 45 degree, move 
#           forward .5m, and return home.
#
#       Much help from James
#
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, Dock, RotateAngle
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration

from action_msgs.msg import GoalStatus

class MyNode(Node):
    """ A class that will create a single node, and reuse a single action client, to move a robot.
    """
    def __init__(self, namespace:str):
        """Initializes MyNode class with a given namespace. Has a single action client

        :param str namespace: what the namespace of robot is
        """
        # call superclass init
        super().__init__('walker')

        print(f"Constructing '{namespace}' node.")
        self._namespace = namespace
        self.result = None

        self._action_client = None
    
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
        while self.result == None:
            rclpy.spin_once(self)

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
        
def main():
    rclpy.init()
    namespace = "create3_05B9"
    node = MyNode(namespace)

    # # undock
    # cur_goal = Undock.Goal()
    # node.send_goal(Undock, 'undock', cur_goal)

    # # drive for 1m
    # cur_goal = DriveDistance.Goal()
    # cur_goal.distance = 1.0
    # node.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # # turn 45deg
    # cur_goal = RotateAngle.Goal()
    # cur_goal.angle = 3.14159/4
    # node.send_goal(RotateAngle, 'rotate_angle', cur_goal)

    # # drive 0.5m
    # cur_goal = DriveDistance.Goal()
    # cur_goal.distance = 0.5
    # node.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # # rotate 130deg
    # cur_goal = RotateAngle.Goal()
    # cur_goal.angle = 3.14159*3/4
    # node.send_goal(RotateAngle, 'rotate_angle', cur_goal)

    # # drive 0.6m
    # cur_goal = DriveDistance.Goal()
    # cur_goal.distance = 0.6
    # node.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # # dock robot
    # cur_goal = Dock.Goal()
    # node.send_goal(Dock, 'dock', cur_goal)


    cur_goal = AudioNoteSequence.Goal()
    # happy sound notes
    notes = [
        AudioNote(frequency=392, max_runtime=Duration(sec=0, nanosec=177500000)),
        AudioNote(frequency=523, max_runtime=Duration(sec=0, nanosec=355000000)),
        AudioNote(frequency=587, max_runtime=Duration(sec=0, nanosec=177500000)),
        AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec=533000000))
    ]

    # sad sound notes
    notes = [
        AudioNote(frequency=82, max_runtime=Duration(sec=1, nanosec=0)),
        AudioNote(frequency=87, max_runtime=Duration(sec=1, nanosec=0))
    ]

    # zelda chest sound notes
    notes = [
        AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=200000000)),
        AudioNote(frequency=466, max_runtime=Duration(sec=0, nanosec=200000000)),
        AudioNote(frequency=494, max_runtime=Duration(sec=0, nanosec=200000000)),
        AudioNote(frequency=523, max_runtime=Duration(sec=1, nanosec=0))
    ]

    # frequencies = [392, 523, 587, 784]
    # notes = [AudioNote() for x in frequencies]
    # for i in range(len(notes)):
    #     notes[i].frequency = frequencies[i]
    #     notes[i].max_runtime = Duration()
    #     notes[i].max_runtime.sec = 0
    #     notes[i].max_runtime.nanosec = 355000000
    note_sequence = AudioNoteVector()
    note_sequence.notes = notes
    cur_goal.iterations = 1
    cur_goal.note_sequence = note_sequence
    node.send_goal(AudioNoteSequence, 'audio_note_sequence', cur_goal)

    # shut down node
    rclpy.shutdown()

if __name__ == '__main__':
    print('--PYTHON SCRIPT--')
    main()