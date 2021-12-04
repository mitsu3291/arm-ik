import math
import numpy as np
from Robot import Robot

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