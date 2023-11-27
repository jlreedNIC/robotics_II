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

        print(f"Constructing '{namespace}' node.")
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
            # print(msg)
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
    
    def rotate_until_dock_found(self):
        print('find dock')
        rotate = 1
        for i in range(3):
            twist = self.create_twist(0, rotate)
            self.publish_msg(twist)
        # find dock
        rotate_counter = 0
        # while self._dock_visible == False and self._opcode == 0:
        while self._opcode != 164 and self._opcode != 168 and self._opcode != 172 and self._opcode != 161:
            # rotate by pub for about 360 degrees
            twist = self.create_twist(0, rotate)
            self.publish_msg(twist)

            rotate_counter += 1
            time.sleep(.5)

            if rotate_counter > 14:
                print('dock not found')
                print(f'opcode: {self._opcode}, sensor: {self._sensor}')
                return False

    def dock(self):
        print('starting pub/sub dock...')

        result = self.rotate_until_dock_found()
        time.sleep(.5)
        if result == False:
            return result
        
        if self._opcode == 161:
            print('dock in force field')
            for i in range(2):
                twist = self.create_twist(0, -1)
                self.publish_msg(twist)
                time.sleep(.5)

            print('move forward')
            twist = self.create_twist(1, 0)
            self.publish_msg(twist)
            time.sleep(.5)

            result = self.rotate_until_dock_found()
            if result == False or self._opcode == 161:
                return result
        
        print(f'opcode: {self._opcode}, sensor: {self._sensor}')
        time.sleep(2)

        print('wiggle until sweet spot')
        # generalize the if statements for wiggle
        buoy_found = self._opcode
        print(f'starting with {buoy_found} buoy')
        for i in range(2):
            print(f'{i}: rotate towards gray')
            while self._opcode != 161 or self._opcode != 172: # or self._sensor != 1:
                twist = self.create_twist(0, 1)
                self.publish_msg(twist)
                time.sleep(.5)
            
            print(f'   rotate towards {buoy_found}')
            while self._opcode != buoy_found or self._opcode != 172: # or self._sensor != 1:
                twist = self.create_twist(0, -1)
                self.publish_msg(twist)
                time.sleep(.5)
        print(f'opcode: {self._opcode}, sensor: {self._sensor}')

        # move forward until in front of dock
        print('moving in front of dock')
        while self._opcode != 172:
            twist = self.create_twist(1, 0)
            self.publish_msg(twist)
            time.sleep(.5)
        
        self.reset_var()
        
        # rotate until sensor 1 and both
        print('rotating to face dock')
        while self._sensor != 1 and self._opcode != 172:
            twist = self.create_twist(0, 1)
            self.publish_msg(twist)
            time.sleep(.5)

        # wiggle until spot is found for both red and green
        # if self._opcode == 168: # red buoy

        #     print('started with red buoy')
        #     counter = 0
        #     # rotate between force field and red 3 times
        #     for i in range(3):
        #         # rotate until force field (161)
        #         print('rotate towards gray')
        #         while self._opcode != 161:
        #             twist = self.create_twist(0, -1)
        #             self.publish_msg(twist)
        #             time.sleep(.5)
        #         # rotate until red buoy (168)
        #         print('rotate towards red')
        #         while self._opcode != 168:
        #             twist = self.create_twist(0, 1)
        #             self.publish_msg(twist)
        #             time.sleep(.5)
        # elif self._opcode == 164: # green buoy
        #     print('started with green buoy')
        #     # rotate between force field and green 3 times
        #     for i in range(3):
        #         # rotate until force field (161)
        #         print('rotate towards gray')
        #         while self._opcode != 161:
        #             twist = self.create_twist(0, 1)
        #             self.publish_msg(twist)
        #             time.sleep(.5)
        #         # rotate until green buoy (164)
        #         print('rotate towards green')
        #         while self._opcode != 164:
        #             twist = self.create_twist(0, -1)
        #             self.publish_msg(twist)
        #             time.sleep(.5)
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