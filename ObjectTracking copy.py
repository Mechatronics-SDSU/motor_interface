class Object_Tracking:

    def follow_object(self, offset_to_follow, current_orientation, current_distance):
        offset = getattr(self.shared_memory_object, offset_to_follow)[0].value
        print("Offset type: {offset}")
        current_orientation = self.shared_memory_object.imu_orientation[0].value
        current_distance = self.shared_memory_object.distance_from_object[0].value

        #NO OBJECT
        if offset == 0 and current_orientation < .1 or current_distance > 1000:
            self.iteration_since_last_detection += 1
            return
        
        #HARD DEADZONE
        #turn right if to the left of the hard deadzone
        elif(offset < -self.x_hard_deadzone):
            self.can.turn_right(abs(offset / self.normalizer_value * self.x_turn_speed))
            self.iteration_since_last_detection = 0
        #turn left if to the right of the hard deadzone
        elif (offset > self.x_hard_deadzone):
            self.can.turn_left(abs(offset / self.normalizer_value * self.x_turn_speed))
            self.iteration_since_last_detection = 0
        
        #SOFT DEADZONE
        #turn right and move forward if to the left of the soft deadzone
        elif (offset < -self.x_soft_deadzone):
            self.can.turn_right(abs(offset / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.speed)
            if (offset == self.shared_memory_object.color_offset[0]):
                self.iteration_since_last_detection = 0
            elif (offset == self.shared_memory_object.gate_offset[0]):
                self.iteration_since_last_detection = 0
                self.iterations += 1

        #turn left and move forward if to the right of the soft deadzone
        elif (offset > self.x_soft_deadzone):
            self.can.turn_left(abs(offset / self.normalizer_value * self.x_turn_speed))
            self.can.move_forward(self.speed)
            if (offset == self.shared_memory_object.color_offset[0]):
                self.iteration_since_last_detection = 0
            elif (offset == self.shared_memory_object.gate_offset[0]):
                self.iteration_since_last_detection = 0
                self.iterations += 1

        #CENTERED
        #move forward if inside soft deadzone    
        else: 
            self.can.move_forward(self.speed)
            if(offset == self.shared_memory_object.yolo_offset[0]):
                #STOP DEPTH
                self.iteration_since_last_detection = 0
                #stop if depth is less than stop value
                if (current_distance < self.distance_stop_value and current_distance != 0.0):
                    print("stop")
                    self.move_forward(1)
                    self.shared_memory_object.enable_color.value = True
                    self.shared_memory_object.enable_yolo.value = False
            elif (offset == self.shared_memory_object.color_offset[0]):
                self.iteration_since_last_detection = 0
            elif ( offset == self.shared_memory_object.gate_offset[0]):
                self.iteration_since_last_detection = 0
                self.iterations += 1
