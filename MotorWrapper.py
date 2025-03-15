import math
import numpy as np
import time
import serial
"""
usbData[0] = motor0; 
usbData[1] = motor1; 
usbData[2] = motor2; 
usbData[3] = motor3; 
usbData[4] = motor4; 
usbData[5] = motor5;
usbData[6] = motor6; 
usbData[7] = motor7; 
usbData[8] = killState; 
usbData[9] = powerOffState; 
usbData[10] = red; 
usbData[11] = green; 
usbData[12] = blue;
"""

class MotorWrapper:

    def __init__(self):

        self.MAX_MOTOR_VAL = 100
    
        #set ~10 for in air, ~30 in water---------------------------------------------------------------
        self.REASONABLE_MOTOR_MAX = 10
        #-------------------------------------------------------------------------------------------------

        self.motors = np.array([
            #LjoyX   LjoyY   RjoyX   RjoyY    Rtrig   Ltrig   LPad       RDpad
            
           # x        y        z        yaw     pitch    roll
            [ 0,      0,       -1,        0,      -1,     -1], # motor 0 (top front left)
            [ 1,      1,        0,       -1,       0,      0], # motor 1 (bottom front left)
            [ 0,      0,       -1,        0,       1,     -1], # motor 2 (top back left)
            [ 1,     -1,        0,       -1,       0,      0], # motor 3 (bottom back left)
            [ 0,      0,       -1,        0,       1,      1], # motor 4 (top back right)
            [ 1,     -1,        0,        1,       0,      0], # motor 5 (bottom back right)
            [ 0,      0,       -1,        0,      -1,      1], # motor 6 (top front right)
            [ 1,      1,        0,        1,       0,      0]  # motor 7 (bottom front right)
        ])

        self.input_list = [0, 0, 0, 0, 0, 0, 0, 0]

    # returns a validated version of the value
    def valid(self, value):
        if type(value) != int: return None # return none if not an int
        # count number of active motors
        count = 0
        for i in range(8):
            if self.input_list[i] != 0:
                count += 1
        # assign max/min
        var = 0
        if count >= 5:
            var = 4200
        elif count >= 3:
            var = 5750
        elif count >= 0:
            var = 7850
        if value > var:
            print("exceeded maximum value")
            return var
        if value < -var:
            print("less than minimum value")
            return -var
        return value

    #custom two's compliment for 2 byte values
    #returns int
    def twos_complement(self, value):
        if (value < 0):
            value = (255 - abs(value))
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
        #translate the direction vector matrix to motor values
        temp_list = np.round(np.dot(matrix, self.motors.transpose()))
        self.input_list += temp_list

    def stop(self): 
        self.input_list = [0,0,0,0,0,0,0,0]

    #sends commands to motors
    def send_command(self):
        motor_value = 0
        command = ""
        for i in range(len(self.input_list)):
            motor_value = int(self.input_list[i])
            motor_value = np.clip(motor_value, -self.REASONABLE_MOTOR_MAX, self.REASONABLE_MOTOR_MAX)
            self.input_list[i] = (motor_value)
            #format command in HEX, getting rid of the first two characters
            #command += '{:02X}'.format(self.twos_complement(motor_value)) + " "

        #init CAN command message
        if self.bus is not None:
            #message = can.Message(arbitration_id = 16, is_extended_id = False, data = bytearray.fromhex(command))
            #self.bus.send(message, timeout = 0.2)
            pass
        else:
            pass
            print(command)

        ret = self.input_list
        self.stop()
        return ret