#----------------------------------------
#
# @file     dock_node.py
# @author   Jordan Reed
# @date     Nov 26, 2023
# @class    Robotics

# @desc     Node to dock the create3 without an action.
#
#
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import irobot_create_msgs
from irobot_create_msgs.msg import DockStatus, IrIntensity, IrIntensityVector, IrOpcode
import time
from nav_msgs.msg import Odometry

import numpy as np


class Docker(Node):
    """ A class that will create a single node, and reuse a single action client, to move a robot.
    """
    def __init__(self, namespace:str):
        """Initializes MyNode class with a given namespace. Has a single action client

        :param str namespace: what the namespace of robot is
        """
        # call superclass init
        super().__init__('docker')

        self.vel_pub = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)

        self.dock_sub = self.create_subscription(DockStatus, f'/{namespace}/dock_status', self.listener, qos_profile_sensor_data)
        self.opcode_sub = self.create_subscription(IrOpcode, f'/{namespace}/ir_opcode', self.listener, qos_profile_sensor_data)
        # self.odom_sub = self.create_subscription(Odometry, f'/{namespace}/odom', self.listener, qos_profile_sensor_data)
        # self.vel_sub = self.create_subscription(Twist, f'/{namespace}/cmd_vel' , self.listener, qos_profile_sensor_data)

        print(f"Constructing '{namespace}' docking node.")
        self._namespace = namespace
        self._is_docked = False
        self._dock_visible = False
        self._opcode = -1
        self._sensor = -1
    
    def listener(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. Here it parses the message from the Robot and updates variables
        '''

        # print(msg)
        try:
            self._opcode = msg.opcode
            self._sensor = msg.sensor
            print(msg)
        except Exception as e:
            # print('not iropcode')
            pass
        
        try:
            self._is_docked = msg.is_docked
            self._dock_visible = msg.dock_visible
        except Exception as e:
            # print('not dock status')
            pass
        # print(f'is docked: {self._is_docked}, dock visible: {self._dock_visible}, opcode: {self._opcode}, sensor: {self._sensor}')
    
    def publish_msg(self, data):
        msg = String()
        msg.data = f'{data}'
        # print(msg)
        # print(type(msg))
        self.vel_pub.publish(data)
    
    def create_twist(self, linear, angular):
        # print('creating twist...')
        # msg_type = "geometry_msgs/msg/Twist"
        # msg = f'"{{linear: {{x: {float(linear)}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {float(angular)}}}}}"'
        twist = Twist()
        twist.linear.x = float(linear)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular)
        return twist

        # return f'{msg_type} {msg}' 

    def reset_var(self):
        self._sensor = -1
        self._opcode = -1
    
    def scan_for_dock(self):
        print('scanning for dock...')

        # opcodes = []
        # sensors = []
        for j in range(2):
            opc = {}

            # do a full 360 scan
            for i in range(13):
                twist = self.create_twist(0, 1)
                self.publish_msg(twist)
                
                time.sleep(.5)
                if self._opcode not in opc.keys():
                    opc[self._opcode] = [self._sensor]
                elif self._sensor not in opc[self._opcode]:
                    opc[self._opcode].append(self._sensor)
                    # opcodes.append(self._opcode)
                    # sensors.append(self._sensor)
        
            # arg = np.argmax(opcodes)
            # best_opcode = opcodes[arg]
            # best_sensor = sensors[arg]

            best_opcode = max(opc.keys())
            best_sensor = max(opc[best_opcode])

            print(f'best opcode: {best_opcode}, best_sensor: {best_sensor}')
            
            if best_opcode == -1:
                print('Dock not found.')
                return False
            elif (best_opcode == 168 or best_opcode == 164) and best_sensor == 1:
                print('Red or Green buoy detected in eyesight.')
                # move forward for set time
                # then rotate to find both red and green in eyesight
                return True
            elif best_opcode == 161 and best_sensor == 1:
                print('force field found in eye sight. Commence movement to better see dock.')
                # rotate until force field found
                # reverse rotate and move forward (move backward)
            elif best_opcode == 161 and best_sensor == 0:
                print('Only force field found in personal bubble. Commence movement to try and find dock again.')
                # rotate until force field found
                # rotate 90 and move forward
                # search again
            elif best_opcode == 168 or best_opcode == 164 and best_sensor == 0:
                print('red or green found in personal space. Back up and try again.')
                # rotate until opcode is found again
                # move backwards
                # try again
        
        # search one more time
        # if red or green found in eyesight, return true
        # else return false
        

    def dock(self):
        print('starting pub/sub dock...')

        result = self.scan_for_dock()
        time.sleep(.5)
        if result == False:
            return False
        # else dock in eyesight
        
        print(f'opcode: {self._opcode}, sensor: {self._sensor}')
        time.sleep(2)

        # print('wiggle')
        # # generalize the if statements for wiggle
        # buoy_found = self._opcode
        # print(f'starting with {buoy_found} buoy')
        # for i in range(2):
        #     print(f'{i}: rotate towards gray')
        #     while self._opcode != 161 or self._opcode != 172: # or self._sensor != 1:
        #         twist = self.create_twist(0, 1)
        #         self.publish_msg(twist)
        #         time.sleep(.5)
            
        #     print(f'   rotate towards {buoy_found}')
        #     while self._opcode != buoy_found or self._opcode != 172: # or self._sensor != 1:
        #         twist = self.create_twist(0, -1)
        #         self.publish_msg(twist)
        #         time.sleep(.5)
        # print(f'opcode: {self._opcode}, sensor: {self._sensor}')

        # # move forward until in front of dock
        # print('moving in front of dock')
        # while self._opcode != 172:
        #     twist = self.create_twist(1, 0)
        #     self.publish_msg(twist)
        #     time.sleep(.5)
        
        # self.reset_var()
        
        # # rotate until sensor 1 and both
        # print('rotating to face dock')
        # while self._sensor != 1 and self._opcode != 172:
        #     twist = self.create_twist(0, 1)
        #     self.publish_msg(twist)
        #     time.sleep(.5)

        
        
        # # move forward slowly until in front of docking station
        # # this logic may need changed
        # # while sensor 0 and force field don't intersect OR
        # # sensor 0 doesn't read both buoys
        # while (self._opcode != 161 and self._sensor != 0) or (self._sensor != 0 and self._opcode != 172):
        #     # move forward slowly 
        #     pass

        # # rotate until sensor 1 and both buoy lined up
        # while self._opcode != 172 and self._sensor != 1:
        #     # rotate
        #     pass

        # move forward until dock not visible and is docked true
        
