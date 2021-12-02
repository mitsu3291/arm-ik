import math

class Robot:
    def __init__(self, l1, l2, q1, q2):
        self.l1 = l1
        self.l2 = l2
        self.q1 = q1
        self.q2 = q2

    def get_joint1_position(self):
        return [self.l1*math.cos(self.q1), self.l1*math.sin(self.q1)]

    def get_joint2_position(self):
        return [self.l1*math.cos(self.q1) + self.l2*math.cos(self.q1 + self.q2),
                self.l1*math.sin(self.q1) + self.l2*math.sin(self.q1 + self.q2)]