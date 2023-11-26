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
        # self.vel_sub = self.create_subscription(Twist, f'/{namespace}/cmd_vel' , self.listener, qos_profile_sensor_data)

        print(f"Constructing '{namespace}' node.")
        self._namespace = namespace
        self._is_docked = False
        self._dock_visible = False
        self._opcode = 0
        self._sensor = 1
    
    def listener(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. Here it parses the message from the Robot and updates variables
        '''

        # print(msg)
        try:
            self._opcode = msg.opcode
            self._sensor = msg.sensor
        except Exception as e:
            print('not iropcode')
            pass
        
        try:
            self._is_docked = msg.is_docked
            self._dock_visible = msg.dock_visible
        except Exception as e:
            print('not dock status')
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

    def dock(self):
        print('starting pub/sub dock...')

        print('find dock')
        # find dock
        rotate_counter = 0
        while self._dock_visible == False:
            # rotate by pub
            twist = self.create_twist(0, 10)
            self.publish_msg(twist)
            rotate_counter += 10

            # if rotate_counter > 300:
            #     return False
        

        print('wiggle until sweet spot')
        # wiggle until spot is found for both red and green
        # if self._opcode == 168: # red buoy
        #     counter = 0
        #     # rotate between force field and red 3 times
        #     for i in range(3):
        #         # rotate until force field (161)
        #         # rotate until red buoy (168)
        #         pass
        # elif self._opcode == 164: # green buoy
        #     # rotate between force field and green 3 times
        #     for i in range(3):
        #         # rotate until force field (161)
        #         # rotate until red buoy (168)
        #         pass
        
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
        

        
        
# def main():
#     rclpy.init()
#     namespace = "create3_05B9"
#     node = Docker(namespace)

#     print('doing something here')

#     import time 
#     time.sleep(2)

#     # shut down node
#     rclpy.shutdown()

# if __name__ == '__main__':
#     print('--PYTHON SCRIPT--')
#     main()