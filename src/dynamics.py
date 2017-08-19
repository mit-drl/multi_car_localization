import numpy as np
from scipy.linalg import block_diag

class Dynamics(object):

    def __init__(self):
        pass

    def state_transition(self, x, u, dt):
        # u is a tuple (u_d, u_a)
        steps = 1.0 #max(int(dt / 0.1),1.0)
        h = dt/float(steps)
        x = [x]
        for i in range(0, int(steps)):
            k1 = self.state_transition_model(x[i], u)
            k2 = self.state_transition_model(x[i] + 0.5*h*k1, u)
            k3 = self.state_transition_model(x[i] + 0.5*h*k2, u)
            k4 = self.state_transition_model(x[i] + k3*h, u)
            new_x = x[i] + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)
            x.append(new_x)
        return x[-1]

    def state_transition_model(self, state, u):
        u_d, u_v, = u
        x, y, phi = state
        dx = u_v*np.cos(phi)
        dy = u_v*np.sin(phi)
        dphi = (u_v/3.)*np.tan(u_d)
        return np.array([dx, dy, dphi])

    def mini_phi(self, state, u, dt):
        theta = state[2]
        v = u[1]
        return np.array([[1, 0, -v*np.sin(theta)*dt],
                         [0, 1, v*np.cos(theta)*dt],
                         [0, 0, 1]])

    def phi(self, state, u, dt, Ncars, Ndim):
        phis = []
        for i in range(Ncars):
            phis.append(self.mini_phi(state[i*Ndim:(i+1)*Ndim], u[i], dt))

        phi = block_diag(*phis)
        return phi

class DubinsVelocityDynamics(Dynamics):

    def __init__(self):
        pass

    def state_transition_model(self, state, u):
        u_d, u_v, = u
        x, y, phi = state
        dx = u_v*np.cos(phi)
        dy = u_v*np.sin(phi)
        dphi = (u_v/3.)*np.tan(u_d)
        return np.array([dx, dy, dphi])

    def mini_phi(self, state, u, dt):
        theta = state[2]
        v = u[1]
        return np.array([[1, 0, -v*np.sin(theta)*dt],
                         [0, 1, v*np.cos(theta)*dt],
                         [0, 0, 1]])


class RoombaDynamics(Dynamics):

    def __init__(self):
        pass

    def state_transition_model(self, state, u):
        u_w, u_v, = u
        x, y, phi = state
        dx = u_v*np.cos(phi)
        dy = u_v*np.sin(phi)
        dphi = u_w
        return np.array([dx, dy, dphi])

    def mini_phi(self, state, u, dt):
        theta = state[2]
        v = u[1]
        return np.array([[1, 0, -v*np.sin(theta)*dt],
                         [0, 1, v*np.cos(theta)*dt],
                         [0, 0, 1]])