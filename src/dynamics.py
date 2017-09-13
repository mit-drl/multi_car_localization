import numpy as np
from scipy.linalg import block_diag

def model(model_name):
    if model_name == "roomba":
        return RoombaDynamics()
    elif model_name == "point":
        return PointDynamics()
    elif model_name == "dubins":
        return DubinsVelocityDynamics()
    else:
        print "INVALID DYNAMICS MODEL NAME GIVEN"

class Dynamics(object):

    def __init__(self):
        pass

    def state_transition(self, x, u, dt):
        # u is a tuple (u_d, u_a)
        steps = 1 #max(int(dt / 0.1),1.0)
        h = dt/float(steps)
        x_i = np.copy(x)
        for i in range(steps):
            k1 = self.state_transition_model(x_i, u)
            k2 = self.state_transition_model(x_i + 0.5*h*k1, u)
            k3 = self.state_transition_model(x_i + 0.5*h*k2, u)
            k4 = self.state_transition_model(x_i + k3*h, u)
            x_i += (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)
        return x_i

    def state_transition_model(self, state, u):
        # must be implemented by subclass
        raise NotImplementedError

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
        self.Ndim = 3
        self.Ninputs = 2

    def state_transition_model(self, state, u):
        u = np.asarray(u)
        state = np.asarray(state)
        u_d, u_v = u[...,0], u[...,1]
        x, y, phi = state[...,0], state[...,1], state[...,2]
        dstate = np.zeros_like(state)
        dstate[...,0] = u_v*np.cos(phi) # dx
        dstate[...,1] = u_v*np.sin(phi) # dy
        dstate[...,2] = (u_v/0.3)*np.tan(u_d) # dphi
        return dstate

    def mini_phi(self, state, u, dt):
        theta = state[2]
        v = u[1]
        return np.array([[1, 0, -v*np.sin(theta)*dt],
                         [0, 1, v*np.cos(theta)*dt],
                         [0, 0, 1]])


class RoombaDynamics(Dynamics):

    def __init__(self):
        self.Ndim = 3
        self.Ninputs = 2

    def state_transition_model(self, state, u):
        u = np.asarray(u)
        state = np.asarray(state)
        u_w, u_v = u[...,0], u[...,1]
        x, y, phi = state[...,0], state[...,1], state[...,2]
        dstate = np.zeros_like(state)
        dstate[...,0] = u_v*np.cos(phi) # dx
        dstate[...,1] = u_v*np.sin(phi) # dy
        dstate[...,2] = u_w # dphi
        return dstate

    def mini_phi(self, state, u, dt):
        theta = state[2]
        v = u[1]
        return np.array([[1, 0, -v*np.sin(theta)*dt],
                         [0, 1, v*np.cos(theta)*dt],
                         [0, 0, 1]])

class PointDynamics(Dynamics):

    def __init__(self):
        self.Ndim = 4
        self.Ninputs = 0

    def state_transition_model(self, state, u):
        # phi is an unused dead state
        state = np.asarray(state)
        x, y, dx, dy = state[...,0], state[...,1], state[...,2], state[...,3]
        dstate = np.zeros_like(state)
        dstate[...,0] = dx
        dstate[...,1] = dy
        return dstate

    def mini_phi(self, state, u, dt):
        return np.array([[1,  0, dt,  0],
                         [0,  1,  0, dt],
                         [0,  0,  1,  0],
                         [0,  0,  0,  1]])
