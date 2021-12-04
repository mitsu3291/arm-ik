import sys
sys.path.append('..')
from utils.PathMaker import PathMaker
from Robot import Robot
from LMIK import LMIK
from Plotter import Plotter
import math

if __name__ == "__main__":
    # Robot Params
    l1 = 1.5
    l2 = 1.5
    l3 = 1.5
    q1 = math.pi  # 特異姿勢を防ぐために最初は0から少しずらす
    q2 = 0.1
    q3 = 0.1

    path_maker = PathMaker()
    line_path = path_maker.make_sin_curve(x_start=-math.pi, x_last=math.pi, offset=2, N=100)

    ik_solver = LMIK(l1=l1, l2=l2, l3=l3, q1=q1, q2=q2, q3=q3, x_ref=1, y_ref=0,
                     eps=1e-4, iter_max=1000, alpha=1.0, damp0=1e-3)
    robot = Robot(l1=l1, l2=l2, l3=l3, q1=q1, q2=q2, q3=q3)

    # Each joint position list
    j1x_pos = []
    j1y_pos = []
    j2x_pos = []
    j2y_pos = []
    j3x_pos = []
    j3y_pos = []

    for i in range(100):
        x_ref = line_path[0][i][0]
        y_ref = line_path[0][i][1]

        # Solve Ik
        ik_solver.set(x_ref=x_ref, y_ref=y_ref)
        q1, q2, q3 = ik_solver.solve()

        # Forward Kinematics
        robot.set(q1=q1, q2=q2, q3=q3)
        j1x, j1y = robot.get_joint1_position()
        j2x, j2y = robot.get_joint2_position()
        j3x, j3y = robot.get_joint3_position()
        j1x_pos.append(j1x)
        j1y_pos.append(j1y)
        j2x_pos.append(j2x)
        j2y_pos.append(j2y)
        j3x_pos.append(j3x)
        j3y_pos.append(j3y)

    # Animation
    plotter = Plotter(j1x_list=j1x_pos, j1y_list=j1y_pos, j2x_list=j2x_pos, j2y_list=j2y_pos, j3x_list=j3x_pos, j3y_list=j3y_pos, ref_path=line_path)
    plotter.animation("3DOF_sin_curve.gif")