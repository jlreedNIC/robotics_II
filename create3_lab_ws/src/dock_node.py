#----------------------------------------
#
# @file     dock_node.py
# @author   Jordan Reed
# @date     Nov 26, 2023
# @class    Robotics
#
# @desc     Node to dock the create3 without an action. Does not work properly.
#
# ------------------------------------------

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import DockStatus, IrOpcode
import time
from nav_msgs.msg import Odometry

class Docker(Node):
    def __init__(self, namespace:str):
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
    
    def create_twist(self, linear, angular):
        """Creates a twist object with a given x:linear and z:angular value. These are the values that will move the create3.
        """
        twist = Twist()
        twist.linear.x = float(linear)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular)
        return twist
  
    def scan_for_dock(self):
        """Scans for the dock using the ir opcodes. Tries to handle many cases.

        :return True or False: Did the robot find the dock with it's 'eyes' or sensor 1
        """
        print('scanning for dock...')

        # do twice
        for j in range(2):
            print(f'starting {j} round')

            # store what robot has seen
            opc = {}

            # do a full 360 scan
            for i in range(13):
                twist = self.create_twist(0, 1)
                self.publish(twist)
                
                time.sleep(.5)
                if self._opcode not in opc.keys():
                    opc[self._opcode] = [self._sensor]
                elif self._sensor not in opc[self._opcode]:
                    opc[self._opcode].append(self._sensor)

            best_opcode = max(opc.keys())
            best_sensor = max(opc[best_opcode])

            if best_opcode == 172 and best_sensor == 0:
                opc.pop(172)

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
                while self._opcode != best_opcode or self._sensor != best_sensor:
                    twist = self.create_twist(0, 1)
                    self.publish(twist)
                    time.sleep(.5)
                return True
            
            elif best_opcode == 172 and best_sensor == 1:
                # rotate until 172 in eyes
                print('putting buoys in eyesight.')
                while True: # self._sensor != 1 and self._opcode != 172:
                    twist = self.create_twist(0, 1)
                    self.publish(twist)
                    time.sleep(.5)

                    if self._sensor == 1 and self._opcode == 172:
                        break
                return True

            elif best_opcode == 161 and best_sensor == 1:
                print('force field found in eye sight. Commence movement to better see dock.')

                # rotate until force field found
                while self._opcode != best_opcode and self._sensor != best_sensor:
                    twist = self.create_twist(0, 1)
                    self.publish(twist)
                    time.sleep(.5)

                # reverse rotate and move forward 
                for i in range(2):
                    twist = self.create_twist(0, -1)
                    self.publish(twist)
                    time.sleep(.5)
                
                # move forward
                for i in range(2):
                    twist = self.create_twist(1,0)
                    self.publish(twist)
                    time.sleep(.5)

            elif best_opcode == 161 and best_sensor == 0:
                print('Only force field found in personal bubble. Commence movement to try and find dock again.')

                # rotate until force field found
                while self._opcode != best_opcode and self._sensor != best_sensor:
                    twist = self.create_twist(0, 1)
                    self.publish(twist)
                    time.sleep(.5)

                # rotate 90 and move forward
                for i in range(6):
                    twist = self.create_twist(0, 1)
                    self.publish(twist)
                    time.sleep(.5)

                # search again
            elif (best_opcode == 168 or best_opcode == 164) and best_sensor == 0:
                print('red or green found in personal space. Back up and try again.')

                # rotate until opcode is found again
                while self._opcode != best_opcode and self._sensor != best_sensor:
                    twist = self.create_twist(0, 1)
                    self.publish(twist)
                    time.sleep(.5)

                print(f'opcode {self._opcode} sensor {self._sensor} found. moving backwards now.')
                # move backwards
                for i in range(2):
                    twist = self.create_twist(-1, 0)
                    self.publish(twist)
                    time.sleep(.5)

                # try again
            print(opc)
        return False

    def dock(self):
        """Complete the docking action. Starts by scanning for the dock, then trying to move in the appropriate way.

        :return True or False: whether or not docking completed
        """
        print('starting pub/sub dock...')

        result = self.scan_for_dock()
        time.sleep(.5)
        print(result)
        if result == False:
            return False
        # else dock in eyesight
        
        print(f'opcode: {self._opcode}, sensor: {self._sensor}')
        time.sleep(2)

        print('sleep done')
        if self._opcode == 164 or self._opcode == 168:
            print('see green or red and moving forward then rotating')
            # move forward for short time
            for i in range(2):
                twist = self.create_twist(1,0)
                self.publish(twist)
                time.sleep(.5)
        
            while self._opcode != 172 or self._opcode != 173:
                twist = self.create_twist(0,1)
                self.publish(twist)
                time.sleep(.5)

        # ready to dock
        if self._opcode == 172 and self._sensor == 1:
            print('---DOCKING---')
            while self._is_docked == False or self._opcode != 173:
                linear = .5
                angular = 0

                # rotate values need tested
                if self._opcode == 164:
                    angular = 1
                elif self._opcode == 168:
                    angular = -1
                
                twist = self.create_twist(linear, angular)
                self.publish(twist)
                time.sleep(1)
            return True

        return False
