import numpy as np

class Driver:
    def __init__(self):
        self.left_correct_intercept = 160
        self.right_correct_intercept = 1250
        self.basic_forward_velocity = 0.1
        self.turning_constant = 0.003

        self.forward_speed_reduction_constant = 10 ** (-1) 

    def controller(self, left_lane_bottom, right_lane_bottom, line_to_follow):
        angular_velocty = 0
        forward_velocity = self.basic_forward_velocity
        if line_to_follow == 'R':
            angular_velocty = self.turning_constant * (self.right_correct_intercept - right_lane_bottom)
            #print("right_lane_bottom", right_lane_bottom)
        else:
            angular_velocty = self.turning_constant * (self.left_correct_intercept - left_lane_bottom)

        if forward_velocity - self.forward_speed_reduction_constant * angular_velocty < 0:
            forward_velocity = 0
        else:
            forward_velocity = forward_velocity - self.forward_speed_reduction_constant * angular_velocty 

        return (forward_velocity, angular_velocty)
