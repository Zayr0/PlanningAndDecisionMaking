import casadi as ca
import numpy as np
import control as ctrl


class Quadrotor:
    def __init__(self):
        self.n_states = 12 # x, y, z, phi, theta, psi, derivatives of before
        self.n_inputs = 4 # F, Tx, Ty, Tz
        self.x = ca.SX.sym('x', self.n_states)  # State variables
        self.u = ca.SX.sym('u', self.n_inputs)  # Input variable
        self.dt = 0.1

        self.build_dyn()  # build the dynamical model

        # linearized state space
        self.A = np.zeros((12, 12))
        self.B = np.zeros((12, 4))
        self.C = np.eye(12) # assume full information feedback
        self.D = np.zeros((12, 4))

        #
        self.dsys = 0
        self.csys = 0

        # build dynamics etc
        x_operating = np.zeros((12, 1))
        u_operating = np.array([10, 0, 0, 0]).reshape((-1, 1))
        self.A, self.B, self.C, self.D = self.linearize(x_operating, u_operating)
        self.K = self.make_dlqr_controller()

    def build_x_dot(self):
        self.x_dot = ca.vertcat(self.x[1], -self.x[0] + self.u)

    def build_dyn(self):
        #self.dyn = ca.Function("bicycle_dynamics_dt_fun", [self.x, self.u], [self.x + self.dt * self.x_dot])
        self.x = ca.SX.sym("x", self.n_states)
        self.u = ca.SX.sym("u", self.n_inputs)

        m = 1  # kg
        g = 9.80665  # m/s^2
        Ix, Iy, Iz = 0.11, 0.11, 0.04  # kg m^2
        l = 0.2  # m (this drops out when controlling via torques)

        # non linear dynamics
        x_x, x_y, x_z, x_phi, x_theta, x_psi, x_dx, x_dy, x_dz, x_dphi, x_dtheta, x_dpsi = ca.vertsplit(self.x, 1)
        u_F, u_Tx, u_Ty, u_Tz = ca.vertsplit(self.u, 1)

        dx_x = x_dx
        dx_y = x_dy
        dx_z = x_dz
        dx_phi = x_dphi
        dx_theta = x_dtheta
        dx_psi = x_dpsi
        dx_dx = u_F/m * (ca.cos(x_phi)*ca.sin(x_theta)*ca.cos(x_psi) + ca.sin(x_phi)*ca.sin(x_psi))
        dx_dy = u_F/m * (ca.cos(x_phi)*ca.sin(x_theta)*ca.sin(x_psi)+ca.sin(x_phi)*ca.cos(x_psi))
        dx_dz = u_F/m * ca.cos(x_phi) * ca.cos(x_theta) - g
        dx_dphi = 1/Ix * (u_Tx + x_dtheta * x_dpsi*(Iy - Iz))
        dx_dtheta = 1/Iy * (u_Ty + x_dpsi*x_dphi*(Iz - Ix))
        dx_dpsi = 1/Iz * (u_Tz + x_dphi*x_dtheta*(Ix-Iy))

        x_dot = ca.vertcat(dx_x, dx_y, dx_z, dx_phi, dx_theta, dx_psi,
                           dx_dx, dx_dy, dx_dz, dx_dphi, dx_dtheta, dx_dpsi)
        self.f = x_dot
        self.dynamics = ca.Function("quadrotor_dyn", [self.x, self.u], [self.x + self.dt * x_dot])

        jac_dyn_x = ca.jacobian(self.x + self.dt * x_dot, self.x)
        jac_dyn_u = ca.jacobian(self.x + self.dt * x_dot, self.u)
        self.jac_dyn_x = ca.Function("jac_dyn_x", [self.x, self.u], [jac_dyn_x])
        self.jac_dyn_u = ca.Function("jac_dyn_u", [self.x, self.u], [jac_dyn_u])

        print("The dynamics were built.")

    def linearize(self, x_operating, u_operating):
        A = ca.Function("A", [self.x, self.u], [ca.jacobian(self.f, self.x)])(x_operating, u_operating)
        B = ca.Function("B", [self.x, self.u], [ca.jacobian(self.f, self.u)])(x_operating, u_operating)
        C = np.eye(12)
        D = np.zeros((self.n_states, self.n_inputs))

        #make numerical matrices and cast into numpy array
        A = np.array(ca.DM(A))
        B = np.array(ca.DM(B))
        return A, B, C, D

    def compute_next_state(self, x, u):
        # x_next = x + self.dt * self.dyn_fun(x, u)
        x_next = self.dynamics(x, u)
        return x_next

    def make_dlqr_controller(self):
        x_operating = np.zeros((12, 1))
        u_operating = np.array([10, 0, 0, 0]).reshape((-1, 1))
        A, B, C, D = self.linearize(x_operating, u_operating)
        Q = np.eye(12)
        R = np.eye(4)
        sys_continuous = ctrl.ss(A, B, C, D)
        self.csys = sys_continuous
        sys_discrete = ctrl.c2d(sys_continuous, self.dt, method='zoh')
        self.dsys = sys_discrete
        K, _, _ = ctrl.dlqr(self.dsys.A, self.dsys.B, Q, R)
        return K

    def get_ss_bag_vectors(self, N):
        """N is the number of simulation steps, thus number of concatinated x vectors"""
        x_bag = np.zeros((self.n_states, N))
        u_bag = np.zeros((self.n_inputs, N))
        return x_bag, u_bag

    def step(self, x, x_ref):
        # discrete step
        x_next = self.dsys.A @ x + self.dsys.B @ self.K @ (x_ref - x)
        return x_next