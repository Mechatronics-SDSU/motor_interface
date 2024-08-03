from motors.MotorWrapper import Can_Wrapper
from multiprocessing import Process, Value
import math
import time
import numpy as np

#from MotorWrapper import Can_Wrapper

class MotorInterface:

    def __init__(self, linear_acceleration_x,   linear_acceleration_y,  linear_acceleration_z, 
                angular_velocity_x,             angular_velocity_y,     angular_velocity_z, 
                orientation_x,                  orientation_y,          orientation_z, 
                distance,
                yolo_offset_x,                  yolo_offset_y,
                dvl_z,
                color_offset_x,                 color_offset_y,
                running,
                enable_yolo,                    enable_color):
        self.linear_acceleration_x = linear_acceleration_x
        self.linear_acceleration_y = linear_acceleration_y
        self.linear_acceleration_z = linear_acceleration_z
        self.angular_velocity_x = angular_velocity_x
        self.angular_velocity_y = angular_velocity_y
        self.angular_velocity_z = angular_velocity_z
        self.orientation_x = orientation_x
        self.orientation_y = orientation_y
        self.orientation_z = orientation_z
        self.distance = distance
        self.yolo_offset_x = yolo_offset_x
        self.yolo_offset_y = yolo_offset_y 
        self.dvl_z = dvl_z
        self.color_offset_x = color_offset_x
        self.color_offset_y = color_offset_y
        self.running = running
        self.min_depth = .9
        self.max_depth = 1.1
        self.enable_color = enable_color
        self.enable_yolo = enable_yolo
        

        self.previous_x_yolo_offsets = []

        self.max_iterations = 10000000
        self.can = Can_Wrapper()

        #TUNE----------------------resize zed camera input-----------------------------------
        self.x_hard_deadzone = 400
        self.y_hard_deadzone = 400

        self.x_soft_deadzone = 50
        self.y_soft_deadzone = 200

        self.x_turn_speed = 5
        self.y_turn_speed = 5
        self.normalizer_value = 640

        self.distance_stop_value = 1000
        self.speed = 20
        self.turn_down_speed = 10

        self.iteration_since_last_detection = 0

    def follow_color(self):            
        #NO OBJECT -------------------------------------------------
        if self.color_offset_x.value == 0:
            self.iteration_since_last_detection += 1
            self.can.stop()
        #HARD DEADZONE----------------------------------------------
        #turn right hard deadzone
        #turn right if to the left of the hard deadzone
        elif(self.color_offset_x.value < -self.x_hard_deadzone):
            self.can.turn_right(abs(self.color_offset_x.value / self.normalizer_value * self.x_turn_speed))
            self.iteration_since_last_detection = 0
        #turn left hard deadzone
        #turn left if to the right of the hard deadzone
        elif (self.color_offset_x.value > self.x_hard_deadzone):
            self.can.turn_left(abs(self.color_offset_x.value / self.normalizer_value * self.x_turn_speed))
            self.iteration_since_last_detection = 0
        #SOFT DEADZONE----------------------------------------------
        #turn right soft deadzone
        #turn right and move forward if to the left of the soft deadzone
        elif (self.color_offset_x.value < -self.x_soft_deadzone):
            self.can.turn_right(abs(self.color_offset_x.value / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.speed)
            self.iteration_since_last_detection = 0
        #turn left soft deadzone
        #turn left and move forward if to the right of the soft deadzone
        elif (self.color_offset_x.value > self.x_soft_deadzone):
            self.can.turn_left(abs(self.color_offset_x.value / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.speed)
            self.iteration_since_last_detection = 0
        #CENTERED---------------------------------------------------
        #move forward if inside soft deadzone    
        else: 
            #print("centered")
            self.can.move_forward(self.speed)
            self.iteration_since_last_detection = 0



    def follow_yolo(self):            
            #NO OBJECT -------------------------------------------------
            if self.yolo_offset_x.value == 0:
                self.iteration_since_last_detection += 1
                self.can.stop()
            #HARD DEADZONE----------------------------------------------
            #turn right hard deadzone
            #turn right if to the left of the hard deadzone
            elif(self.yolo_offset_x.value < -self.x_hard_deadzone):
                self.can.turn_right(abs(self.yolo_offset_x.value / self.normalizer_value * self.x_turn_speed))
                self.iteration_since_last_detection = 0
            #turn left hard deadzone
            #turn left if to the right of the hard deadzone
            elif (self.yolo_offset_x.value > self.x_hard_deadzone):
                self.can.turn_left(abs(self.yolo_offset_x.value / self.normalizer_value * self.x_turn_speed))
                self.iteration_since_last_detection = 0
            #SOFT DEADZONE----------------------------------------------
            #turn right soft deadzone
            #turn right and move forward if to the left of the soft deadzone
            elif (self.yolo_offset_x.value < -self.x_soft_deadzone):
                self.can.turn_right(abs(self.yolo_offset_x.value / self.normalizer_value * self.x_turn_speed))
                self.can.move_forward(self.speed)
                self.iteration_since_last_detection = 0
            #turn left soft deadzone
            #turn left and move forward if to the right of the soft deadzone
            elif (self.yolo_offset_x.value > self.x_soft_deadzone):
                self.can.turn_left(abs(self.yolo_offset_x.value / self.normalizer_value * self.x_turn_speed))
                self.can.move_forward(self.speed)
                self.iteration_since_last_detection = 0
            #CENTERED---------------------------------------------------
            #move forward if inside soft deadzone    
            else: 
                #print("centered")
                self.can.move_forward(self.speed)
                self.iteration_since_last_detection = 0
            #STOP DEPTH-----------------------------------------------
            #stop if depth is less than stop value
            if (self.distance.value < self.distance_stop_value and self.distance.value != 0.0):
                print("stop")
                self.can.stop()

    def move_down(self, down_time):
        self.can.move_down(20)
        self.can.send_command()
        time.sleep(down_time)

    def move_forward(self, forward_time):
        self.can.move_forward(20)
        self.can.move_down(.3)
        self.can.send_command()
        time.sleep(forward_time)

    def look_for_detection(self):
        #turn left if nothing in front
        if self.yolo_offset_x.value == 0:
        #if self.yolo_offset_x.value == 0:
            self.can.turn_right(10) #lower turn speed?
            self.can.send_command()
            time.sleep(.3)
            self.can.stop()
            self.can.send_command()
            time.sleep(1)
        
        if self.yolo_offset_x.value != 0:
            self.iteration_since_last_detection = 0
        else:
            self.iteration_since_last_detection += 1

    def sit_at_depth(self):
        # print(self.dvl_z.value)
        if (self.dvl_z.value <= 0.0):
            return
        if self.dvl_z.value < self.min_depth:
            self.can.move_down(.2)
        elif self.dvl_z.value > self.max_depth:
            self.can.move_up(.4)
        pass

    def run_loop(self):

        # self.move_down(2)
        # self.can.stop()
        # self.can.send_command()

        while self.running.value:
            start = time.time()

            self.sit_at_depth()

            if self.enable_color.value:
                self.follow_color()

            elif self.enable_yolo.value:
                self.follow_yolo()
            # if (self.iteration_since_last_detection < 20):
            #     print(self.iteration_since_last_detection)
            #     s            # if (self.iteration_since_last_detection < 20):
            #     print(self.iteration_since_last_detection)
            #     self.follow()
            # else:
            #     self.look_for_detection()
            #     print(self.iteration_since_last_detection)elf.follow()
            # else:
            #     self.look_for_detection()
            #     print(self.iteration_since_last_detection)
            end = time.time()
            time.sleep(.05 - (end - start))
            self.can.send_command()


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
    yolo_offset_x = Value('d', 0.0)
    yolo_offset_y = Value('d', 0.0)
    color_offset_x = Value('d', 0.0)
    color_offset_y = Value('d', 0.0)
    color = Value('i', 0)


    interface = MotorInterface(
        linear_acceleration_x   = lin_acc_x,
        linear_acceleration_y   = lin_acc_y,
        linear_acceleration_z   = lin_acc_z,
        angular_velocity_x      = ang_vel_x,
        angular_velocity_y      = ang_vel_y,
        angular_velocity_z      = ang_vel_z,
        orientation_x           = orientation_x,
        orientation_y           = orientation_y,
        orientation_z           = orientation_z,
        distance                = depth,
        yolo_offset_x           = yolo_offset_x,
        yolo_offset_y           = yolo_offset_y,
        dvl_z                   = depth,
        color_offset_x          = color_offset_x,
        color_offset_y          = color_offset_y
    )
                
    yolo_offset_x.value = 0.0
    depth.value = 100   

    interface.run_loop()
