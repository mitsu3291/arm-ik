import math
import numpy as np
import sys
sys.path.append('..')
from utils.PathMaker import PathMaker
from Robot import Robot
from Plotter import Plotter
from Plotter import simple_plot

class LMIK:
    def __init__(self, l1, l2, l3, q1, q2, q3, x_ref, y_ref, eps, iter_max, alpha, damp0):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.q1 = q1
        self.q2 = q2 
        self.q3 = q3
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.eps = eps
        self.iter_max = iter_max
        self.alpha = alpha
        self.damp0 = damp0

    def set(self, x_ref, y_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref

    def get_jacobian(self):
        s1 = math.sin(self.q1)
        s12 = math.sin(self.q1+self.q2)
        s123 = math.sin(self.q1+self.q2+self.q3)
        c1 = math.cos(self.q1)
        c12 = math.cos(self.q1+self.q2)
        c123 = math.cos(self.q1+self.q2+self.q3)
        J = np.array([[-self.l1*s1-self.l2*s12-self.l3*s123, -self.l2*s12-self.l3*s123, -self.l3*s123],
                      [self.l1*c1+self.l2*c12+self.l3*c123, self.l2*c12+self.l3*c123, self.l3*c123]])

        return J

    def solve(self):
        robot = Robot(l1=self.l1, l2=self.l2, l3=self.l3, q1=self.q1, q2=self.q2, q3=self.q3)
        
        i=0
        while True:
            x_ref = np.array([self.x_ref, self.y_ref])
            robot.set(q1=self.q1, q2=self.q2, q3=self.q3)
            x3, y3 = robot.get_joint3_position()
            x_now = np.array([x3, y3])
            err = x_ref - x_now
            if np.linalg.norm(err) < self.eps: # 誤差が小さければ反復終了
                Is_success = True
                break
            if i >= self.iter_max:
                Is_success = False
                break
            J = self.get_jacobian()
            damp = self.damp0 + np.linalg.norm(err)
            dq = J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(2), err)) # Levenberg-Marquardt 法で q の更新ステップを決定．
            self.q1 += dq[0]
            self.q2 += dq[1]
            self.q3 += dq[2]
            if not i % 10:
                print('%d: error = %s' % (i, err.T))
            i += 1

        if Is_success:
            print("Convergence achieved!")
        else:
            print("Warning: the iterative algorithm has not reached convergence to the desired precision")

        return self.q1, self.q2, self.q3

if __name__ == "__main__":
    # Robot Params
    l1 = 0.5
    l2 = 0.5
    l3 = 0.5
    q1 = 0.1  # 特異姿勢を防ぐために最初は0から少しずらす
    q2 = 0.1
    q3 = 0.1

    path_maker = PathMaker()
    line_path = path_maker.make_lines(x_start=1, y_start=-1, x_last=1, y_last=1, N=100)

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
    plotter.animation("3DOF_line_path.gif")