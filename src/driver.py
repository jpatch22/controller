import numpy as np

class Driver:
    def __init__(self):
        self.left_correct_intercept = 250
        self.right_correct_intercept = 1100
        self.left_basic_forward_velocity = 0.2
        self.right_basic_forward_velocity = 0.2
        self.left_turning_constant = 0.007
        self.right_turning_constant = 0.007
        self.right_turn_k_deriv = 0.0004 * 4

        self.left_forward_speed_reduction_constant = 10 ** (-1) 
        self.right_forward_speed_reduction_constant = 10 ** (-1) 

        self.right_previous_error = 0

    def controller(self, main_intercept, turning_intercept, line_to_follow):
        if line_to_follow == 'R':
            forward_velocity, angular_velocty = self.follow_right_line(main_intercept, turning_intercept)
        else:
            forward_velocity, angular_velocty = self.follow_left_line(main_intercept, turning_intercept)
        return (forward_velocity, angular_velocty)

    def follow_right_line(self, main_intercept, turning_intercept):
        angular_velocty = 0
        forward_velocity = self.right_basic_forward_velocity
        error = self.right_correct_intercept - main_intercept
        derivative = error - self.right_previous_error
        self.right_previous_error = error
        # print("Error: {}, Deriv: {}".format(error, derivative))
        # print("TURNING LEFT", turning_intercept)

        angular_velocty = self.right_turning_constant * error + self.right_turn_k_deriv * derivative

        if forward_velocity - self.right_forward_speed_reduction_constant * angular_velocty < 0:
            forward_velocity = 0
        else:
            forward_velocity = forward_velocity - self.right_forward_speed_reduction_constant * angular_velocty
        
        return (forward_velocity, angular_velocty) 

    def follow_left_line(self, main_intercept, turning_intercept):
        angular_velocty = 0
        forward_velocity = self.left_basic_forward_velocity

        angular_velocty = self.left_turning_constant * (self.left_correct_intercept - main_intercept)

        if forward_velocity - self.left_forward_speed_reduction_constant * angular_velocty < 0:
            forward_velocity = 0
        else:
            forward_velocity = forward_velocity - self.left_forward_speed_reduction_constant * angular_velocty

        return (forward_velocity, angular_velocty) 

