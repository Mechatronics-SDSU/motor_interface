
import math
import numpy as np
import time

try:
    import can
except:
    print("CAN library not installed")

'''
    discord: @kialli
    github: @kchan5071
    
    This class is a wrapper for the CAN bus interface. It is used to send commands to the motors.
    contains:
        move methods for motors
        twos compliment conversion

        test sequence if called as main
'''


class Can_Wrapper:

    def __init__(self):
        #set CAN device
        self.bus = None
        try:
            self.bus = can.Bus(interface='socketcan',channel = 'can0', receive_own_messages=True)
        except:
            print("CAN device not found")

        self.MAX_MOTOR_VAL = 100
    
        #set ~10 for in air, ~30 in water---------------------------------------------------------------
        self.REASONABLE_MOTOR_MAX = 10
        #-------------------------------------------------------------------------------------------------

        self.motors = np.array([
            #LjoyX   LjoyY   RjoyX   RjoyY    Rtrig   Ltrig   LPad       RDpad
            
           # x        y        z        yaw     pitch    roll
            [ 0,      0,       -1,        0,      -1,     -1], # motor 0 (top front left)
            [-1,      1,        0,       -1,       0,      0], # motor 1 (bottom front left)
            [ 0,      0,       -1,        0,       1,     -1], # motor 2 (top back left)
            [-1,     -1,        0,       -1,       0,      0], # motor 3 (bottom back left)
            [ 0,      0,       -1,        0,       1,      1], # motor 4 (top back right)
            [-1,     -1,        0,        1,       0,      0], # motor 5 (bottom back right)
            [ 0,      0,       -1,        0,      -1,      1], # motor 6 (top front right)
            [-1,      1,        0,        1,       0,      0]  # motor 7 (bottom front right)
        ])
        self.input_list = [0, 0, 0, 0, 0, 0, 0, 0]

    #custom two's compliment for 2 byte values
    #returns int
    def twos_complement(self, value):
        if (value < 0):
            value = 255 - abs(value)
        return value

    def move_forward(self, value):
        self.move_from_matrix(np.array([value, 0, 0, 0, 0, 0]))

    def move_backward(self, value):
        self.move_from_matrix(np.array([-value, 0, 0, 0, 0, 0]))

    def move_left(self, value):
        self.move_from_matrix(np.array([0, value, 0, 0, 0, 0]))

    def move_right(self, value):
        self.move_from_matrix(np.array([0, -value, 0, 0, 0, 0]))

    def move_up(self, value):
        self.move_from_matrix(np.array([0, 0, value, 0, 0, 0]))

    def move_down(self, value):
        self.move_from_matrix(np.array([0, 0, -value, 0, 0, 0]))

    def turn_up(self, value):
        self.move_from_matrix(np.array([0, 0, 0, value, 0, 0]))

    def turn_down(self, value):
        self.move_from_matrix(np.array([0, 0, 0, -value, 0, 0]))

    def turn_left(self, value):
        self.move_from_matrix(np.array([0, 0, 0, 0, value, 0]))

    def turn_right(self, value):
        self.move_from_matrix(np.array([0, 0, 0, 0, -value, 0]))

    def roll_left(self, value):
        self.move_from_matrix(np.array([0, 0, 0, 0, 0, value]))

    def roll_right(self, value):
        self.move_from_matrix(np.array([0, 0, 0, 0, 0, -value]))

    def move_from_matrix(self, matrix):
        temp_list = np.dot(matrix, self.motors.transpose())
        for i in range(len(temp_list)):
            temp_list[i] = np.clip(temp_list[i], -self.REASONABLE_MOTOR_MAX, self.REASONABLE_MOTOR_MAX)
        self.input_list = temp_list


    def stop(self):
        self.input_list = [0,0,0,0,0,0,0,0]


    #sends commands to motors
    def send_command(self):
        motor_value = 0
        command = ""
        for motor_value_from_list in self.input_list:
            motor_value = self.twos_complement(int(motor_value_from_list))
            #format command in HEX, getting rid of the first two characters
            command += '{:02X}'.format(motor_value) + " "

        #init CAN command message
        if self.bus is not None:
            message = can.Message(arbitration_id = 16, is_extended_id = False, data = bytearray.fromhex(command))
            self.bus.send(message, timeout = 0.2)
        else:
            pass
            print(command)
            print(self.input_list)

        self.stop()