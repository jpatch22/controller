class Driver:
    def __init__(self):
        self.alignment_constant = 0.0001
        self.direction_constant = 0.00003
        self.basic_forward_velocity = 0.02

    def controller(self, heading_line, width):
        print('HL', heading_line)
        direction_term = self.alignment_constant * (heading_line[0][0] - heading_line[0][2])
        alignment_term = self.alignment_constant * (heading_line[0][2] + heading_line[0][1] - width)

        angular_velocty = direction_term

        return (self.basic_forward_velocity, angular_velocty)
