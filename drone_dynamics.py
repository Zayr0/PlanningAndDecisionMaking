import Casadi as ca
import numpy as np

class Quadrotor:
    def __int__(self):
        self.n_states = 12 # x, y, z, phi, theta, psi, derivatives of before
        self.n_inputs = 4 # F, Tx, Ty, Tz
        self.x = ca.MX.sym('x', self.n_states)  # State variables
        self.u = ca.MX.sym('u', self.n_inputs)  # Input variable
        self.t = ca.MX.sym('t')  # Time

        # linearized state space
        self.A = np.zeros((12, 12))
        self.B = np.zeros((12, 4))
        self.C = np.eye(12) # assume full information feedback
        self.D = np.zeros((12, 4))

    def build_x_dot(self):
        self.x_dot = ca.vertcat(self.x[1], -self.x[0] + self.u)

    def build_dyn(self):
        self.dyn = ca.Function("bicycle_dynamics_dt_fun", [self.x, self.u], [self.x + self.dt * self.x_dot])
        x = ca.SX.sym("x", self.n_states)
        u = ca.SX.sym("u", self.n_inputs)

        # TODO: specify constants
        x_x, x_y, x_z, x_phi, x_theta, x_psi, x_dx, x_dy, x_dz, x_dphi, x_dtheta, x_dpsi = ca.vertsplit(x, 1)
        dx_x = x_dx
        dx_y = x_dy
        dx_z = x_dz
        dx_phi = x_dphi
        dx_theta = x_dtheta
        dx_psi = x_dpsi
        dx_dx = 1/m * (ca.cos(x_phi)*ca.sin(x_theta)*ca.cos(x_psi) + ca.sin(x_phi)*ca.sin(x_psi))
        dx_dy = F/m * (ca.cos(x_phi)*ca.sin(x_theta)*ca.sin(x_psi)+ca.sin(x_phi)*ca.cos(x_psi))
        dx_dz = F/m * ca.cos(x_phi) * ca.cos(x_theta) - m * g
        dx_dphi = 1/Ix * ((F1-F3)*l + x_dtheta * x_dpsi*(Iy- Iz))
        dx_dtheta = 1/Iy 
        dx_dpsi
        u_F, u_Tx, u_Ty, u_Tz = ca.vertsplit(u, 1)

    def compute_next_state(self, x, u):
        # x_next = x + self.dt * self.dyn_fun(x, u)
        x_next = self.dyn_dt_fun(x, u)
        return x_next

    def f(self, x):
        x_x, x_y, x_z, x_theta, x_phi, x_psi,dx_x, dx_y, dx_z, dx_theta, dx_phi, dx_psi = ca.vertsplit(x, 1)
        f1 = v * ca.cos(theta)
        f2 = v * ca.sin(theta)
        f3 = v * ca.tan(phi) / self._L
        f4 = ca.SX.zeros(1)
        f5 = ca.SX.zeros(1)
        f = ca.vertcat(f1, f2, f3, f4, f5)
        return f