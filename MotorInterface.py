from motors.MotorWrapper            import Can_Wrapper
from multiprocessing                import Process, Value
from motors.ObjectTracking          import Object_Tracking
import math
import time
import numpy as np

#from MotorWrapper import Can_Wrapper

class MotorInterface:

    def __init__(self, shared_memory_object, x_hard_deadzone):
        self.shared_memory_object = shared_memory_object
        
        self.previous_x_yolo_offsets = []

        # self.max_iterations = 10000000
        self.can = Can_Wrapper()

        #TUNE----------------------resize zed camera input-----------------------------------
        self.x_hard_deadzone = x_hard_deadzone
        self.y_hard_deadzone = 400

        self.x_soft_deadzone = 200
        self.y_soft_deadzone = 200

        self.x_turn_speed = 5
        self.y_turn_speed = 5
        self.normalizer_value = 640

        self.distance_stop_value = 1000
        self.speed = 20
        self.turn_down_speed = 5

        self.is_looking_for_detection = True
        self.corrected_drift = False

        self.iterations = 0
        self.max_iterations = 100 + 200
        #timeout / detection parameters-------------------------------------------------------

        self.iteration_since_last_detection = 0
        self.iterations_before_detection_timeout = 100

        self.detection_thrust_length = 10
        self.detection_thrust_count = 0

        self.wait_length = 10
        self.current_wait = 0

    def look_for_detection(self):
        if self.shared_memory_object.lin_ang_vel[0].value > self.y_turn_speed:
            return
        if self.detection_thrust_count <= self.detection_thrust_length:
            self.can.turn_right(1)
            self.detection_thrust_count += 1
            return
        elif self.current_wait < self.wait_length:
            self.can.stop()
            self.current_wait += 1
            return
        else:
            self.current_wait = 0
            self.detection_thrust_count = 0

    def sit_at_depth(self):
        if self.shared_memory_object.depth.value < self.min_depth:
            self.can.move_down(.5)
        elif self.shared_memory_object.depth.value > self.max_depth:
            self.can.move_up(.4)
        pass

    def face_direction(self, target):
        if self.shared_memory_object.imu_orientation[1].value < -.001:
            self.can.turn_left(min(abs(target - self.shared_memory_object.imu_orientation[1].value) * 3, 1))
            print("Correcting to the left")
        elif self.shared_memory_object.imu_orientation[1].value > .001:
            self.can.turn_right(min(abs(target - self.shared_memory_object.imu_orientation[1].value) * 3, 1))   
            print("Correcting to the right")  

    def correct_pitch(self):
        if self.shared_memory_object.imu_orientation[0].value < -.001:
            self.can.turn_up(min(abs(self.shared_memory_object.imu_orientation[0].value) * 3, 1))
        elif self.shared_memory_object.imu_orientation[0].value > .001:
            self.can.turn_down(min(self.shared_memory_object.imu_orientation[0].value * 3, 1))

    def correct_drift(self):
        if self.shared_memory_object.depth.value > self.min_depth - .1:
            return
        if self.shared_memory_object.imu_orientation[1].value < -.1:
            self.can.turn_left(self.shared_memory_object.imu_orientation[1].value / 2)
            print("turn left")
        elif self.shared_memory_object.imu_orientation[1].value > .1:
            self.can.turn_right(self.shared_memory_object.imu_orientation[1].value / 2)
            print("turn right")
        if self.shared_memory_object.imu_orientation[0].value < -.1:
            self.can.turn_up(self.shared_memory_object.imu_orientation[0].value / 2)
            print("turn up")
        elif self.shared_memory_object.imu_orientation[0].value > .1:
            self.can.turn_down(self.shared_memory_object.imu_orientation[0].value / 2)
            print("turn down")
        if self.shared_memory_object.imu_orientation[2].value < -.1:
            self.can.roll_left(self.shared_memory_object.imu_orientation[2].value / 2)
            print("roll left")
        elif self.shared_memory_object.imu_orientation[2].value > .1:
            self.can.roll_right(self.shared_memory_object.imu_orientation[2].value / 2)
            print("roll right")    
        self.corrected_drift = True   

    def run_loop(self):
        self.iterations = Object_Tracking.iterations()
        self.iteration_since_last_detection = Object_Tracking.iteration_since_last_detection()
        while self.shared_memory_object.imu_lin_acc[0].value == 0.0: #wait for imu to start
            pass
        while self.iterations < 100 and self.shared_memory_object.running.value:
            start = time.time()
            self.sit_at_depth()
            self.face_direction(0)
            self.correct_pitch()
            end = time.time()
            if (.05 - (end - start)) > 0:
                time.sleep(.05 - (end - start))
            self.can.send_command()
            self.iterations += 1

        while self.shared_memory_object.running.value:   
            start = time.time()
            if (self.iteration_since_last_detection > self.iterations_before_detection_timeout) and self.enable_yolo:
                self.look_for_detection()
            else:
                self.current_wait = 0
                self.detection_thrust_count = 0
            if self.shared_memory_object.gate_enable.value:
                print("Offset: ", self.shared_memory_object.color_offset[0])
                self.Object_Tracking.follow_gate()
            if self.shared_memory_object.enable_yolo.value:
                self.iterations += 1
                self.look_for_detection()
                self.Object_Tracking.follow_buoy()
                
            self.sit_at_depth()
            print("Sitting at depth")
            if self.iterations == self.max_iterations and self.iterations - self.max_iterations < 50:
                self.can.move_forward(3)
                self.shared_memory_object.enable_color.value = False
                self.shared_memory_object.color_offset[0].value = 0
                pass
            if self.iterations == self.max_iterations:
                self.can.stop()
                self.shared_memory_object.enable_yolo.value = True
                self.shared_memory_object.enable_color.value = False
            end = time.time()
            if (.05 - (end - start)) > 0:
                time.sleep(.05 - (end - start))
            self.can.send_command()

