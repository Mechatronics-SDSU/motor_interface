from Motor.MotorWrapper import  Can_Wrapper
from multiprocessing import     Process,    Value
import math
import time



#from MotorWrapper import Can_Wrapper

class MotorInterface:

    '''
        discord: @kialli
        github: @kchan5071
        
        creates shared memory and processes to communicate between vision and control
        
        vision: writes to shared memory
        currently just printing process data, later to use with PID control
        
        could use arrays, but prefer to use shared memory for simplicity and readability
        
    '''

    def __init__(self, linear_acceleration_x ,  linear_acceleration_y,  linear_acceleration_z, 
                angular_velocity_x,             angular_velocity_y,     angular_velocity_z, 
                orientation_x,                  orientation_y,          orientation_z, 
                depth,
                offset_x,                       offset_y):
        self.linear_acceleration_x = linear_acceleration_x
        self.linear_acceleration_y = linear_acceleration_y
        self.linear_acceleration_z = linear_acceleration_z
        self.angular_velocity_x = angular_velocity_x
        self.angular_velocity_y = angular_velocity_y
        self.angular_velocity_z = angular_velocity_z
        self.orientation_x = orientation_x
        self.orientation_y = orientation_y
        self.orientation_z = orientation_z
        self.depth = depth
        self.offset_x = offset_x
        self.offset_y = offset_y 

        self.max_iterations = 10000
        self.can = Can_Wrapper()

        #TUNE---------------------------------------------------------
        self.x_hard_deadzone = 40
        self.y_hard_deadzone = 40

        self.x_soft_deadzone = 20
        self.y_soft_deadzone = 20

        self.x_turn_speed = .1
        self.y_turn_speed = .1

        self.depth_stop_value = 60
        self.speed = .05


    def run_loop(self):
        for i in range(self.max_iterations):
            print(self.offset_x.value)
            print(self.offset_y.value)
            print(self.depth)
            #no object
            if (self.offset_x.value == 0 and self.offset_y.value == 0):
                self.can.stop()
            #HARD DEADZONE----------------------------------------------
            #turn right hard deadzone
            if (self.offset_x.value > self.x_hard_deadzone):
                self.can.turn_right(self.x_turn_speed)
            #turn left hard deadzone
            if (self.offset_x.value < -self.x_hard_deadzone):
                self.can.turn_left(self.x_turn_speed)
            #turn up hard deadzone
            if (self.offset_y.value > self.y_hard_deadzone):
                self.can.turn_up(self.y_turn_speed)
            
            #turn down hard deadzone
            if (self.offset_y.value < -self.y_hard_deadzone):
                self.can.turn_down(self.y_turn_speed)
            
            #SOFT DEADZONE----------------------------------------------
            #turn left soft deadzone
            elif (self.offset_x.value < -self.x_soft_deadzone):
                self.can.turn_left(self.x_turn_speed)
                self.can.move_forward(self.speed)
            
            #turn right soft deadzone
            elif (self.offset_x.value > self.x_soft_deadzone):
                self.can.turn_right(self.x_turn_speed)
                self.can.move_forward(self.speed)
            
            #turn up soft deadzone
            if (self.offset_y.value > self.y_soft_deadzone):
                self.can.turn_up(self.y_turn_speed)
                self.can.move_forward(self.speed)
            
            #turn down soft deadzone
            elif (self.offset_y.value < -self.y_soft_deadzone):
                self.can.turn_down(self.y_turn_speed)
            
            #STOP DEPTH-----------------------------------------------
            #stop if depth is less than stop value
            if (self.depth.value < self.depth_stop_value):
                self.can.stop()
            
            #MOVE FORWARD---------------------------------------------
            self.can.move_forward(self.speed)
            self.can.send_command()
            time.sleep(.05)


if __name__ == '__main__':
    ang_vel_x = Value('d', 0.0)
    ang_vel_y = Value('d', 0.0)
    ang_vel_z = Value('d', 0.0)
    lin_acc_x = Value('d', 0.0)
    lin_acc_y = Value('d', 0.0)
    lin_acc_z = Value('d', 0.0)
    orientation_x = Value('d', 0.0)
    orientation_y = Value('d', 0.0)
    orientation_z = Value('d', 0.0)
    depth = Value('d', 0.0)
    offset_x = Value('d', 0.0)
    offset_y = Value('d', 0.0)

    interface = MotorInterface(linear_acceleration_x=lin_acc_x, linear_acceleration_y=lin_acc_y, linear_acceleration_z=lin_acc_z,        #linear accel x y z
                    angular_velocity_x=ang_vel_x, angular_velocity_y=ang_vel_y, angular_velocity_z=ang_vel_z,               #angular velocity x y z
                    orientation_x=orientation_x, orientation_y=orientation_y, orientation_z=orientation_z,                  #orientation x y z
                    depth=depth,                                                                                            #depth
                    offset_x=offset_x, offset_y=offset_y)  
                
    offset_x.value = 100.0
    depth.value = 100   

    interface.run_loop()