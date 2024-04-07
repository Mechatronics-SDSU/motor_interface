import can
import math
import numpy as np
import time

# This class is a wrapper for the CAN bus interface. It is used to send commands to the motors.
# The class has methods to move the robot in different directions and to stop the robot.
# The class also has a method to send the commands to the motors.
# The class has a method to clamp the motor values to a reasonable range.
# The class has a method to convert a value to two's complement.


class Can_Wrapper:

    def __init__(self):
        self.bus = can.Bus(interface='socketcan',channel = 'can0', receive_own_messages=True)

        self.MAX_MOTOR_VAL = 100
    
        #set ~10 for in air, ~30 in water---------------------------------------------------------------
        self.REASONABLE_MOTOR_MAX = 10
        #-------------------------------------------------------------------------------------------------

        self.motors = [
            #LjoyX   LjoyY   RjoyX   RjoyY    Rtrig   Ltrig   LPad       RDpad
            
           #strafe for/bck   roll    ptch#     up#    down#   Lroll#     Rroll#
            [ 0,      0,      -1,     -1,      -1,      1,      1,        -1], # motor 0 (top front left)
            [ 1,     -1,       0,      0,       0,      0,      0,         0], # motor 1 (bottom front left)
            [ 0,      0,      -1,      1,      -1,      1,      1,        -1], # motor 2 (top back left)
            [-1,     -1,       0,      0,       0,      0,      0,         0], # motor 3 (bottom back left)
            [ 0,      0,       1,      1,      -1,      1,     -1,         1], # motor 4 (top back right)
            [-1,     -1,       0,      0,       0,      0,      0,         0], # motor 5 (bottom back right)
            [ 0,      0,       1,     -1,      -1,      1,     -1,         1], # motor 6 (top front right)
            [ 1,     -1,       0,      0,       0,      0,      0,         0]  # motor 7 (bottom front right)
        ]
        self.input_list = [0, 0, 0, 0, 0, 0, 0, 0]

    def clamp(self, num):
        ret = max(-1 * self.REASONABLE_MOTOR_MAX, num)
        ret = min(self.REASONABLE_MOTOR_MAX, num)
        return ret


    def twos_complement(self, value):
        if (value < 0):
            value = 255 - abs(value)
        return value

    def move_forward(self, value):
        self.input_list[1] = self.clamp(self.input_list[1] + value)

    def move_backward(self, value):
        self.input_list[1] = self.clamp(self.input_list[1] + -value)

    def move_left(self, value):
        self.input_list[0] = self.clamp(self.input_list[0] + value)

    def move_right(self, value):
        self.input_list[0] = self.clamp(self.input_list[0] + -value)

    def move_up(self, value):
        self.input_list[4] = self.clamp(self.input_list[4] + value)

    def move_down(self, value):
        self.input_list[5] = self.clamp(self.input_list[5] + value)

    def turn_up(self, value):
        self.input_list[3] = self.clamp(self.input_list[3] + value)

    def turn_down(self, value):
        self.input_list[4] = self.clamp(self.input_list[4] + -value)

    def turn_left(self, value):
        self.input_list[2] = self.clamp(self.input_list[2] + value)

    def turn_right(self, value):
        self.input_list[2] = self.clamp(self.input_list[2] + -value)


    def stop(self):
        self.input_list = [0,0,0,0,0,0,0,0]


    def send_command(self):
        thrust_list = []
        for motor in self.motors:
            thrust_list.append(int(self.REASONABLE_MOTOR_MAX * np.dot(motor, self.input_list)))

        motor_value = 0
        command = ""
        for motor_value_from_list in thrust_list:
            motor_value = self.twos_complement(int(motor_value_from_list))
            command += '{:02X}'.format(motor_value) + " "

        message = can.Message(arbitration_id = 16, is_extended_id = False, data = bytearray.fromhex(command))
        self.bus.send(message, timeout = 0.2)

#main for testing
def main():
    wrapper = Can_Wrapper()
    #-----------------------------forward
    wrapper.move_forward(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.move_backward(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------backward
    wrapper.move_backward(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.stop()
    wrapper.send_command()
    time.sleep(1)



    #----------------------------left
    wrapper.move_left(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.move_right(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------right
    wrapper.move_right(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.stop()
    wrapper.send_command()
    time.sleep(1)


    #-----------------------------up
    wrapper.move_up(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.move_down(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------down
    wrapper.move_down(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.stop()
    wrapper.send_command()
    time.sleep(1)


    #-----------------------------pitch up
    wrapper.turn_up(.1)
    wrapper.send_command()
    time.sleep(1)

    #-----------------------------stop
    wrapper.turn_down(.1)
    wrapper.send_command()
    time.sleep(1)

    #-----------------------------pitch down
    wrapper.turn_down(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.stop()
    wrapper.send_command()
    time.sleep(1)



    #-----------------------------turn left
    wrapper.turn_left(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.turn_right(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------turn right
    wrapper.turn_right(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------stop
    wrapper.stop()
    wrapper.send_command()
    time.sleep(1)





    #-----------------------------stop
    wrapper.stop()
    wrapper.send_command()




if __name__ == "__main__":
    main()