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

#def random_joint(joint_range):
import random

def get_random_position(j1:list, j2:list, j3:list, j4:list, j5:list, j6:list):
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

    arm_up = [-103.6, 34.7, 77.1, -189.8, -10.8, 0.7]
    arm_down = [-103.6, 82.8, 23.3, -181.3, -10.8, 0.7]

    for i in range(4):
        larry_node.send_joint_pose_goal(arm_up)
        larry_node.send_joint_pose_goal(arm_down)

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
        larry_node.send_joint_offset_goal(5, 20)
        larry_node.send_joint_offset_goal(5, -20)

def larry_sprinkler(larry_node:MyNode):
    print('send goal to make larry do the sprinkler')

def larry_disco(larry_node:MyNode):
    print('send goal to make larry do disco moves')

def main():
    rclpy.init()
    Larry = MyNode()

    # positions
    j1_range = [-100,100]
    j2_range = [-100,100]
    j3_range = [-100,100]
    j4_range = [-100,100]
    j5_range = [-100,100]
    j6_range = [-100,100]
    joint_home = [-95.8, 48.7, 40.6, -169.3, -8.6, 0.7]
    
    first_move = get_random_position(j1_range, j2_range, j3_range, j4_range, j5_range, j6_range)
    print(first_move)

    # # attempt to make while loop better
    # try:
    #     # put robot move code here
    #     pass
    # except KeyboardInterrupt:
    #     # ------
    #     # reset robot
    #     # ------

    #     print('KeyboardInterrupt, sending robot home and shutting down.')

    #     Larry.send_joint_pose_goal(joint_home)   # go back to home position

    #     # shut down node
    #     Larry.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    print('--PYTHON SCRIPT--')
    main()