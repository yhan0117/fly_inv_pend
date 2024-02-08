import numpy as np
from typing import List

# Numerical simulation of the pendulum
class pend():
    def __init__(self):
        self.l = 1
        self.m = 1
        self.g = 9.8
        self.dt = 0.1

    # Convert motor commands and quadrotor states to linear accelerations of the quadrotor's CG
    def get_accel(self, cmd_motor, x) -> List[int]:

    # Equations of motion for forward integration
    def eom(self, accel_trans, x) -> List[int]:
    '''
        accel_trans: translational acceleration of quadrotor, 1x3
        x: current pendulum states, 1x4
    '''
        dx = np.zeros(4)
        # 1st order state transition
        dx[0] = x[2] 
        dx[1] = x[3]

        # 2nd order state transition (derived via Lagrangian dynamics)
        dx[2] = (accel_trans[1]*np.cos(x[0]) + accel_trans[2]*np.sin(x[0]) + 2*self.l*x[2]*x[3]*np.sin(x[1]) + self.g*np.sin(x[0])) / (self.l*np.cos(x[1]))
        dx[3] = (-accel_trans[0]*np.cos(x[1]) - accel_trans[1]*np.sin(x[0])*np.sin(x[1]) + accel_trans[2]*np.cos(x[0])*np.sin(x[1]) + self.l*x[2]**2*np.cos(x[0])*np.sin(x[0]) + self.g*np.cos(x[0])*np.sin(x[1])) / (self.l)

        return dx.tolist()

    # Runge Kutta 4th order integrator for the DAE
    def rk4(self, cmd_motor, x):
        # obtain quadrotor acceleration 
        accel_trans = self.get_accel(cmd_motor, x)

        # integration
        k1 = self.dt*np.array(eom(accel_trans, x))
        k2 = self.dt*np.array(eom(accel_trans, x))


