import MotorWrapper

class MotorInterface:

    def __init__(self, linear_acceleration_x , linear_acceleration_y, linear_acceleration_z, 
                angular_velocity_x, angular_velocity_y, angular_velocity_z, 
                orientation_x, orientation_y, orientation_z, 
                depth,
                offset_x, offset_y):
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
        self.iterations = 0


    def run_loop(self):
        
        while (self.iterations < self.max_iterations):
            if (self.offset_x.value > 0):
                MotorWrapper.move_right(.1)
                MotorWrapper.send_command()
                return
            elif (self.offset_x.value < 0):
                MotorWrapper.move_left(.1)
                MotorWrapper.send_command()
                return
            if (self.offset_y.value > 0):
                MotorWrapper.move_up(.1)
                MotorWrapper.send_command()
                return
            elif (self.offset_y.value < 0):
                MotorWrapper.move_down(.1)
                MotorWrapper.send_command()
                return

            MotorWrapper.move_forward(.1)
            MotorWrapper.send_command()
            self.iterations += 1

    

        
