# ---------------------------
# @file     CS554_reed_lab3.py
# @author   Jordan Reed
# @date     9/26/23
# @brief    Program to move dice around using CRX10's and newly developed
#           Python API.
#
# @class    CS 554 - Lab #3
# ---------------------------

import sys

try:
    from robot_controller import robot
    print('import worked as is')
except Exception as e:
    sys.path.append("FANUC-Ethernet_IP_Drivers/")
    from robot_controller import robot

# initialize robot
ip_address = '129.101.98.214' # Bill
# ip_address = '129.101.98.215' # DJ

try:
    myrobot = robot(ip_address)
except Exception as e:
    print('Error connecting to robot.')
    print(e)
    exit()

# positions in joint
home = [0,0,0,0,0,0]
approach_di_home = [22.93075180053711,17.527952194213867,-60.5017204284668,-27.101898193359375,63.29799270629883,-168.42613220214844]
pick_up_di_home = [23.265966415405273,29.93922233581543,-76.43579864501953,-24.962177276611328,77.68882751464844,-175.70835876464844]
approach_conveyor_start = [-52.32429504394531,20.60003089904785,-39.86384201049805,64.18165588378906,60.3193359375,-224.9647979736328]
drop_conveyor_start = [-51.13820266723633,26.970178604125977,-52.76607894897461,57.38116455078125,65.92820739746094,-211.89108276367188]
approach_conveyor_end = [-67.2178726196289,-3.4131338596343994,-21.83332061767578,-49.80992126464844,29.67200469970703,-136.071533203125]
pick_up_conveyor_end = [-63.82355499267578,6.01451301574707,-42.89634704589844,-36.04237365722656,47.27680969238281,-155.71011352539062]

# ----------
# program start
# ----------

# initialize robot
print('Initializing robot...')
myrobot.conveyor('stop') # make sure conveyor is not running
myrobot.onRobot_gripper_close(110, 10) # make sure gripper is open
myrobot.set_pose(home)
myrobot.start_robot()

# approach di
myrobot.set_speed(250)
myrobot.set_pose(approach_di_home)
myrobot.start_robot()

# pick up di
myrobot.set_pose(pick_up_di_home)
myrobot.start_robot()

# close gripper
myrobot.onRobot_gripper_close(76, 10)

# approach di
myrobot.set_pose(approach_di_home)
myrobot.start_robot()

# hover above conveyor
myrobot.set_pose(approach_conveyor_start)
myrobot.start_robot()

# drop on conveyor
myrobot.set_pose(drop_conveyor_start)
myrobot.start_robot()

# open gripper
myrobot.onRobot_gripper_close(110, 10)

# hover above conveyor
myrobot.set_pose(approach_conveyor_start)
myrobot.start_robot()

# move robot out of way
# done in non-blocking manner so conveyor will start at same time
myrobot.set_pose(approach_conveyor_end)
myrobot.start_robot(blocking=False)

# conveyor loop
print('Starting conveyor...')
myrobot.conveyor('forward')
while(myrobot.conveyor_proximity_sensor('left') == 0):
    pass
myrobot.conveyor('stop')

# go to pick up dice
myrobot.set_pose(pick_up_conveyor_end)
myrobot.start_robot()

# close gripper
myrobot.onRobot_gripper_close(76, 10)

# hover over conveyor
myrobot.set_pose(approach_conveyor_end)
myrobot.start_robot()

# rotate dice and move over starting spot
myrobot.set_pose(approach_di_home)
myrobot.start_robot()

# set dice down
myrobot.set_pose(pick_up_di_home)
myrobot.start_robot()

# let go of di and open gripper
myrobot.onRobot_gripper_close(110, 10)

# hover above di
myrobot.set_pose(approach_di_home)
myrobot.start_robot()

# go back to home
myrobot.set_pose(home)
myrobot.start_robot()
