import Casadi as ca
import numpy as np


class Quadrotor:
    def __int__(self):
        self.n_states = 12 # x, y, z, phi, theta, psi, derivatives of before
        self.n_inputs = 4 # F, Tx, Ty, Tz
        self.x = ca.MX.sym('x', self.n_states)  # State variables
        self.u = ca.MX.sym('u', self.n_inputs)  # Input variable
        self.dt = 0.1

        # linearized state space
        self.A = np.zeros((12, 12))
        self.B = np.zeros((12, 4))
        self.C = np.eye(12) # assume full information feedback
        self.D = np.zeros((12, 4))

    def build_x_dot(self):
        self.x_dot = ca.vertcat(self.x[1], -self.x[0] + self.u)

    def build_dyn(self):
        #self.dyn = ca.Function("bicycle_dynamics_dt_fun", [self.x, self.u], [self.x + self.dt * self.x_dot])
        x = ca.SX.sym("x", self.n_states)
        u = ca.SX.sym("u", self.n_inputs)

        # TODO: specify constants
        m = 1  # kg
        g = 9.80665  # m/s^2
        Ix, Iy, Iz = 0.11, 0.11, 0.04  # kg m^2
        l = 0.2  # m (this drops out when controlling via torques)

        # non linear dynamics
        x_x, x_y, x_z, x_phi, x_theta, x_psi, x_dx, x_dy, x_dz, x_dphi, x_dtheta, x_dpsi = ca.vertsplit(x, 1)
        u_F, u_Tx, u_Ty, u_Tz = ca.vertsplit(u, 1)

        dx_x = x_dx
        dx_y = x_dy
        dx_z = x_dz
        dx_phi = x_dphi
        dx_theta = x_dtheta
        dx_psi = x_dpsi
        dx_dx = 1/m * (ca.cos(x_phi)*ca.sin(x_theta)*ca.cos(x_psi) + ca.sin(x_phi)*ca.sin(x_psi))
        dx_dy = u_F/m * (ca.cos(x_phi)*ca.sin(x_theta)*ca.sin(x_psi)+ca.sin(x_phi)*ca.cos(x_psi))
        dx_dz = u_F/m * ca.cos(x_phi) * ca.cos(x_theta) - m * g
        dx_dphi = 1/Ix * (u_Tx + x_dtheta * x_dpsi*(Iy- Iz))
        dx_dtheta = 1/Iy * (u_Ty + x_dpsi*x_dphi*(Iz-Ix))
        dx_dpsi = 1/Iz * (u_Tz + x_dphi*x_dtheta*(Ix-Iy))

        x_dot = ca.vertcan(dx_x, dx_y, dx_z, dx_phi, dx_theta, dx_psi,
                           dx_dx, dx_dy, dx_dz, dx_dphi, dx_dtheta, dx_dpsi)
        self.dyns = ca.Function("quadrotor_dyn", [x, u], [x + self.dt * x_dot])

        jac_dyn_x = ca.jacobian(x + self.dt * x_dot, x)
        jac_dyn_u = ca.jacobian(x + self.dt * x_dot, u)
        self.jac_dyn_x = ca.Function("jac_dyn_x", [x, u], [jac_dyn_x])
        self.jac_dyn_u = ca.Function("jac_dyn_u", [x, u], [jac_dyn_u])


        print("The dynamics were built.")

    def compute_next_state(self, x, u):
        # x_next = x + self.dt * self.dyn_fun(x, u)
        x_next = self.dyn_dt_fun(x, u)
        return x_next

    def compute_next_state_approx(self, x, u, x_r, u_r):
        """x_r and u_r are the reference points"""
        dfdx = self.jac_dyn_x(x_r, u_r)
        dfdu = self.jac_dyn_u(x_r, u_r)
        x_next = self.dyns(x, u) + dfdx @ (x - x_r) + dfdu @ (u - u_r)
        return x_next