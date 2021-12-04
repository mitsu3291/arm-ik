import math

class Robot:
    def __init__(self, l1, l2, l3, q1, q2, q3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def set(self, q1, q2, q3):
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def get_joint1_position(self):
        x1 = self.l1*math.cos(self.q1)
        y1 = self.l1*math.sin(self.q1)
        return x1, y1

    def get_joint2_position(self):
        x2 = self.l1*math.cos(self.q1) + self.l2*math.cos(self.q1 + self.q2)
        y2 = self.l1*math.sin(self.q1) + self.l2*math.sin(self.q1 + self.q2)
        return x2, y2

    def get_joint3_position(self):
        x3 = self.l1*math.cos(self.q1) + self.l2*math.cos(self.q1 + self.q2) + self.l3*math.cos(self.q1 + self.q2 + self.q3)
        y3 = self.l1*math.sin(self.q1) + self.l2*math.sin(self.q1 + self.q2) + self.l3*math.sin(self.q1 + self.q2 + self.q3)
        return x3, y3