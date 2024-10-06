from motors.MotorWrapper import Can_Wrapper
import time

class MotorInterface:

    def __init__(self, shared_memory_object, x_hard_deadzone):

        self.shared_memory_object = shared_memory_object
        self.can = Can_Wrapper()


        #----------------------Tune-----------------------------------

        #--------------------Deadzones--------------------------------
        # Unit: number of pixels from the center of an image.
        # When looking for the gate, if the center of the gate is outside of these values, turn towards gate only.
        self.x_hard_deadzone = x_hard_deadzone
        self.y_hard_deadzone = 400
        # When looking for the gate, if the center of the gate is outside of these values,
        # but inside the hard_deadzone values, turn towards gate and move forward.
        # If inside of these values, move foward only.
        self.x_soft_deadzone = 200
        self.y_soft_deadzone = 200

        self.move_forward_speed = 20
        self.x_turn_speed = 5
        self.y_turn_speed = 5
        self.turn_down_speed = 5

        # Half of the horizontal resolution of the image.
        # Used for converting from a pixel offset from the center to a ratio from 0 to 1.
        self.normalizer_value = 640

        # The maximum distance from gate after shooting to it to change from the gate task to the buoy task. 
        self.distance_stop_value = 1000

        self.iterations = 0
        # self.max_iterations = 10000000
        self.max_iterations = 100 + 200
        #timeout / detection parameters------------------------------

        self.iteration_since_last_detection = 0
        self.iterations_before_detection_timeout = 100

        self.detection_thrust_length = 10
        self.detection_thrust_count = 0

        self.wait_length = 10
        self.current_wait = 0

    def follow_buoy(self):   
        print(self.shared_memory_object.color_offset[0].value )         
        #NO OBJECT -------------------------------------------------
        if self.shared_memory_object.color_offset[0].value == 0 and self.shared_memory_object.imu_orientation[0].value < .1:
            self.iteration_since_last_detection += 1
            return
        #HARD DEADZONE----------------------------------------------
        #turn right hard deadzone
        #turn right if to the left of the hard deadzone
        elif(self.shared_memory_object.color_offset[0].value < -self.x_hard_deadzone):
            self.can.turn_right(abs(self.shared_memory_object.color_offset[0].value / self.normalizer_value * self.x_turn_speed / 3))
            self.iteration_since_last_detection = 0
        #turn left hard deadzone
        #turn left if to the right of the hard deadzone
        elif (self.shared_memory_object.color_offset[0].value > self.x_hard_deadzone):
            self.can.turn_left(abs(self.shared_memory_object.color_offset[0].value / self.normalizer_value * self.x_turn_speed / 3))
            self.iteration_since_last_detection = 0
        #SOFT DEADZONE----------------------------------------------
        #turn right soft deadzone
        #turn right and move forward if to the left of the soft deadzone
        elif (self.shared_memory_object.color_offset[0].value < -self.x_soft_deadzone):
            self.can.turn_right(abs(self.shared_memory_object.color_offset[0].value / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.move_forward_speed)
            self.iteration_since_last_detection = 0
        #turn left soft deadzone
        #turn left and move forward if to the right of the soft deadzone
        elif (self.shared_memory_object.color_offset[0].value > self.x_soft_deadzone):
            self.can.turn_left(abs(self.shared_memory_object.color_offset[0].value / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.move_forward_speed)
            self.iteration_since_last_detection = 0
        #CENTERED---------------------------------------------------
        #move forward if inside soft deadzone    
        else: 
            #print("centered")
            self.can.move_forward(self.move_forward_speed)
            self.iteration_since_last_detection = 0

    def follow_gate(self):   
        print("Offset x", self.shared_memory_object.gate_offset[0].value )         
        #NO OBJECT -------------------------------------------------
        if self.shared_memory_object.gate_offset[0].value == 0 and self.shared_memory_object.imu_orientation[0].value < .1:
            self.iteration_since_last_detection += 1
            return
        #HARD DEADZONE----------------------------------------------
        # #turn right hard deadzone
        # #turn right if to the left of the hard deadzone
        elif(self.shared_memory_object.gate_offset[0].value < -self.x_hard_deadzone):
            self.can.turn_right(abs(self.shared_memory_object.gate_offset[0].value / self.normalizer_value * self.x_turn_speed))
            self.iteration_since_last_detection = 0
        # #turn left hard deadzone
        # #turn left if to the right of the hard deadzone
        elif (self.shared_memory_object.gate_offset[0].value > self.x_hard_deadzone):
            self.can.turn_left(abs(self.shared_memory_object.gate_offset[0].value / self.normalizer_value * self.x_turn_speed))
            self.iteration_since_last_detection = 0
        #SOFT DEADZONE----------------------------------------------
        #turn right soft deadzone
        #turn right and move forward if to the left of the soft deadzone
        elif (self.shared_memory_object.gate_offset[0].value < -self.x_soft_deadzone):
            self.can.turn_right(abs(self.shared_memory_object.gate_offset[0].value / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.move_forward_speed)
            self.iteration_since_last_detection = 0
            self.iterations += 1
        #turn left soft deadzone
        #turn left and move forward if to the right of the soft deadzone
        elif (self.shared_memory_object.gate_offset[0].value > self.x_soft_deadzone):
            self.can.turn_left(abs(self.shared_memory_object.gate_offset[0].value / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.move_forward_speed)
            self.iteration_since_last_detection = 0
            self.iterations += 1
        #CENTERED---------------------------------------------------
        #move forward if inside soft deadzone    
        else: 
            #print("centered")
            self.can.move_forward(self.move_forward_speed)
            self.iteration_since_last_detection = 0
            self.iterations += 1



    def follow_yolo(self):            
            #NO OBJECT -------------------------------------------------
            if self.shared_memory_object.yolo_offset[0].value == 0.0 or self.shared_memory_object.distance_from_object.value > 10000:
                self.iteration_since_last_detection += 1
                return
            #HARD DEADZONE----------------------------------------------
            #turn right hard deadzone
            #turn right if to the left of the hard deadzone
            elif(self.shared_memory_object.yolo_offset[0].value < -self.x_hard_deadzone):
                self.can.turn_right(abs(self.shared_memory_object.yolo_offset[0].value / self.normalizer_value * self.x_turn_speed))
            #turn left hard deadzone
            #turn left if to the right of the hard deadzone
            elif (self.shared_memory_object.yolo_offset[0].value > self.x_hard_deadzone):
                self.can.turn_left(abs(self.shared_memory_object.yolo_offset[0].value / self.normalizer_value * self.x_turn_speed))
            #SOFT DEADZONE----------------------------------------------
            #turn right soft deadzone
            #turn right and move forward if to the left of the soft deadzone
            elif (self.shared_memory_object.yolo_offset[0].value < -self.x_soft_deadzone):
                self.can.turn_right(abs(self.shared_memory_object.yolo_offset[0].value / self.normalizer_value * self.x_turn_speed))
                self.can.move_forward(self.move_forward_speed)
            #turn left soft deadzone
            #turn left and move forward if to the right of the soft deadzone
            elif (self.shared_memory_object.yolo_offset[0].value > self.x_soft_deadzone):
                self.can.turn_left(abs(self.shared_memory_object.yolo_offset[0].value / self.normalizer_value * self.x_turn_speed))
                self.can.move_forward(self.move_forward_speed)
            #CENTERED---------------------------------------------------
            #move forward if inside soft deadzone    
            else: 
                #print("centered")
                self.can.move_forward(self.move_forward_speed)
            #STOP DEPTH-----------------------------------------------
            self.iteration_since_last_detection = 0
            #stop if depth is less than stop value
            if (self.shared_memory_object.distance_from_object.value < self.distance_stop_value and self.shared_memory_object.distance_from_object.value != 0.0):
                print("stop")
                self.move_forward(1)
                self.shared_memory_object.enable_color.value = True
                self.shared_memory_object.enable_yolo.value = False


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
        # print(self.shared_memory_object.depth.value)
        if self.shared_memory_object.depth.value < self.min_depth:
            self.can.move_down(.5)
        elif self.shared_memory_object.depth.value > self.max_depth:
            self.can.move_up(.4)
        pass

    def face_direction(self, target):
        #print(target - self.orientation_y.value)
        if self.shared_memory_object.imu_orientation[1].value < -.001:
            self.can.turn_left(min(abs(target - self.shared_memory_object.imu_orientation[1].value) * 3, 1))
            print("Correcting to the left")
        elif self.shared_memory_object.imu_orientation[1].value > .001:
            self.can.turn_right(min(abs(target - self.shared_memory_object.imu_orientation[1].value) * 3, 1))   
            print("Correcting to the right")  

    def correct_pitch(self):
        # print(self.shared_memory_object.imu_orientation[0].value)
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

    def run_loop(self):

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
                self.follow_gate()
            if self.shared_memory_object.enable_yolo.value:
                self.iterations += 1
                self.look_for_detection()
                self.follow_buoy()
                
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

