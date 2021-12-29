import math
import numpy as np
from Robot import Robot
from casadi import *

class IKCon:
    def __init__(self, l1, l2, l3, q1, q2, q3, x_ref, y_ref, eps, iter_max, alpha, damp0, omega_max, dt):
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
        self.omega_max = omega_max
        self.dt = dt

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

    def solve(self, is_first):
        opti = Opti() # Initialize optimization problem

        # Set decision variables
        q = opti.variable(3) # joint angles
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        
        # Forward kinematics
        x = self.l1*cos(q1) + self.l2*cos(q1 + q2) + self.l3*cos(q1 + q2 + q3)
        y = self.l1*sin(q1) + self.l2*sin(q1 + q2) + self.l3*sin(q1 + q2 + q3)

        # Objective function
        opti.minimize((x - self.x_ref)**2 + (y - self.y_ref)**2)

        # Set constraints
        if not is_first:
            opti.subject_to(q1-self.q1 <= self.omega_max*self.dt)
            opti.subject_to(q1-self.q1 >= -self.omega_max*self.dt)
            opti.subject_to(q2-self.q2 <= self.omega_max*self.dt)
            opti.subject_to(q2-self.q2 >= -self.omega_max*self.dt)
            opti.subject_to(q3-self.q3 <= self.omega_max*self.dt)
            opti.subject_to(q3-self.q3 >= -self.omega_max*self.dt)

        # Set initial guess
        opti.set_initial(q1, self.q1)
        opti.set_initial(q2, self.q2)
        opti.set_initial(q3, self.q3)

        # Solve Problem
        opti.solver("ipopt")
        sol = opti.solve() 

        # Set joint angle
        self.q1 = sol.value(q1)
        self.q2 = sol.value(q2)
        self.q3 = sol.value(q3)

        return self.q1, self.q2, self.q3