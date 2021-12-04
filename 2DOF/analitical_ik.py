import math
import sys
sys.path.append('..')
from utils.PathMaker import PathMaker
from Robot import Robot
from Plotter import Plotter

class AnaliticalIk:
    def __init__(self, l1, l2, x_ref, y_ref):
        self.l1 = l1
        self.l2 = l2
        self.x_ref = x_ref
        self.y_ref = y_ref

    def set(self, x_ref, y_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref

    def solve(self):
        alpha = math.acos((self.x_ref**2 + self.y_ref**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2))
        beta = math.acos((self.x_ref**2 + self.y_ref**2 + self.l1**2 - self.l2**2)/(2*self.l1*math.sqrt(self.x_ref**2 + self.y_ref**2)))
        q1 = math.atan2(self.y_ref, self.x_ref) + beta
        q2 = math.atan2(self.y_ref - self.l1*math.sin(q1), self.x_ref - self.l1*math.cos(q1)) - q1

        return q1, q2

if __name__ == "__main__":
    # Robot parameter
    l1 = 0.8
    l2 = 0.8

    path_maker = PathMaker()
    line_path = path_maker.make_lines(x_start=1, y_start=-1, x_last=1, y_last=1, N=100)
    ik_solver = AnaliticalIk(l1=l1, l2=l2, x_ref=1, y_ref=-1)
    robot = Robot(l1=l1, l2=l2, q1=0, q2=0)

    # Each joint position list
    j1x_pos = []
    j1y_pos = []
    j2x_pos = []
    j2y_pos = []

    for i in range(100):
        x_ref = line_path[0][i][0]
        y_ref = line_path[0][i][1]

        # Solve Ik
        ik_solver.set(x_ref=x_ref, y_ref=y_ref)
        q1, q2 = ik_solver.solve()

        # Forward Kinematics
        robot.set(q1=q1, q2=q2)
        j1x, j1y = robot.get_joint1_position()
        j2x, j2y = robot.get_joint2_position()
        j1x_pos.append(j1x)
        j1y_pos.append(j1y)
        j2x_pos.append(j2x)
        j2y_pos.append(j2y)

    # Animation
    plotter = Plotter(j1x_list=j1x_pos, j1y_list=j1y_pos, j2x_list=j2x_pos, j2y_list=j2y_pos, ref_path=line_path)
    plotter.animation()