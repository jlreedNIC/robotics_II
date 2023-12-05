#-------------------------------------------
#
# @file     main.py
# @author   Jordan Reed
# @date     November, 2023
# @brief    Program to test out the ROS2 driver designed by James Lasso. 
#           Moves FANUC P50ib or 'Larry'
#
# ------------------------------------------

import rclpy
from ros2_node import MyNode

import random

def get_random_position(j1:list, j2:list, j3:list, j4:list, j5:list, j6:list):
    """Generate a random joint position within the ranges given.

    :param list j1: minimum and maximum of joint 1
    :param list j2: minimum and maximum of joint 2
    :param list j3: minimum and maximum of joint 3
    :param list j4: minimum and maximum of joint 4
    :param list j5: minimum and maximum of joint 5
    :param list j6: minimum and maximum of joint 6
    :return _type_: list of joint positions to pass to robot
    """
    if len(j1) != 2 and len(j2) != 2 and len(j3) != 2 and len(j4) != 2 and len(j5) != 2 and len(j6) != 2:
        print('All ranges must be of length 2.')
        return
    
    pose = [
        random.randint(j1[0], j1[1]),
        random.randint(j2[0], j2[1]),
        random.randint(j3[0], j3[1]),
        random.randint(j4[0], j4[1]),
        random.randint(j5[0], j5[1]),
        random.randint(j6[0], j6[1])
    ]

    return pose

def larry_jab(larry_node:MyNode):
    print(f'send goal to perform the "jab" move')

    # coordinates of 'jab' movement
    # arm_up = [-103.6, 34.7, 77.1, 20.0, -10.8, 0.7]
    # arm_down = [-103.6, 82.8, 77.1, 20.0, -10.8, 0.7]

    # for i in range(2):
    #     larry_node.send_joint_pose_goal(arm_up)
    #     larry_node.send_joint_pose_goal(arm_down)

    # relatively same code as above but in joint offset movements
    for i in range(2):
        larry_node.send_joint_offset_goal(2, -10)

        larry_node.send_joint_offset_goal(2, 10)
        larry_node.send_joint_offset_goal(2, 10)
    
    print('jab move done')

def larry_swinging_nod(larry_node:MyNode):
    print('send goal to make larry do the "swinging nod" move')
    home = [-95.8, 48.7, 40.6, -169.3, -8.6, 0.7]
    j1_range = [-94.0, 5.0]
    j3_range = [29.0, 52.0]

    # 3 positions
    left = [min(j1_range)] + [home[2]] + [min(j3_range)] + home[3:6]
    mid = [(j1_range[0] + j1_range[1])/2] + [home[2]] + [(j3_range[0]+j3_range[1])/2] + home[3:6]
    right = [max(j1_range)] + [home[2]] + [max(j3_range)] + home[3:6]

    for i in range(4):
        larry_node.send_joint_pose_goal(left)
        larry_node.send_joint_pose_goal(mid)
        larry_node.send_joint_pose_goal(right)
        larry_node.send_joint_pose_goal(mid)

def larry_wave(larry_node:MyNode):
    print('send goal to larry to do a wave')

    for i in range(4):
        larry_node.send_joint_offset_goal(3, 10)
        larry_node.send_joint_offset_goal(3, 10)
        larry_node.send_joint_offset_goal(3, -10)
        larry_node.send_joint_offset_goal(3, -10)

def larry_sprinkler(larry_node:MyNode):
    print('send goal to make larry do the sprinkler')
    print('not currently implemented')

def larry_disco(larry_node:MyNode):
    print('send goal to make larry do disco moves')
    print('not currently implemented')

def go_home(larry_node:MyNode):
    """Send larry to custom defined home position by one joint move at a time

    :param MyNode larry_node: Node corresponding to Larry's movements
    """
    joint_home = [-95.8, 48.7, 40.6, 5.0, -8.6, 0.7]

    for i in range(0, len(joint_home)):
        print(f'doing joint {i+1} - {joint_home[i]}')
        larry_node.send_joint_position_goal(i+1, joint_home[i])

def main():
    rclpy.init()
    Larry = MyNode()

    # unused code for random positions
    # positions
    j1_range = [-100,100]
    j2_range = [-100,100]
    j3_range = [-100,100]
    j4_range = [-100,100]
    j5_range = [-100,100]
    j6_range = [-100,100]
    joint_home = [-95.8, 48.7, 40.6, -169.3, -8.6, 0.7]

    try:
        # Larry.send_joint_pose_goal(joint_home)    # go back to home position by passing pose
        # go_home(Larry)                            # go back home one joint at a time

        # larry_jab(Larry)              # dance move
        # larry_swinging_nod(Larry)     # dance move
        # larry_wave(Larry)             # dance move
        print('done')
        
        go_home(Larry)                              # back to home
        # Larry.send_joint_pose_goal(joint_home)    # go back to home position
        pass
    except KeyboardInterrupt:
        # ------
        # reset robot
        # ------

        print('KeyboardInterrupt, sending robot home and shutting down.')

        # Larry.send_joint_pose_goal(joint_home)   # go back to home position

        # shut down node
        Larry.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    print('--PYTHON SCRIPT--')
    main()