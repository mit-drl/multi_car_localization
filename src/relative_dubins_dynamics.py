import numpy as np
from scipy.stats import norm
from numpy.random import multivariate_normal
from utils import rk4
from utils import euler
import pdb
import time


class RelativeDubinsDynamics(object):

    def __init__(self, Ncars, initial_transforms, noise_u, noise_uwb):
        self.initial_transforms = initial_transforms
        self.Ncars = Ncars
        self.noise_u = noise_u
        self.noise_uwb = noise_uwb

        # for simulation
        self.state = np.reshape(initial_transforms, ((self.Ncars - 1) * 3, 1))

    def dynamics(self, T, u):
        # T: a N x d matrix
        # u: a N x m matrix

        if u.ndim == 1:
            u = u.reshape((1, u.shape[0]))
        dT = np.zeros(T.shape)

        dT[:, 0::3] = np.multiply(u[:, 2::2], np.cos(T[:, 2::3])) + np.multiply(T[:, 1::3], u[:, 1, None]) - u[:, 0, None]
        dT[:, 1::3] = np.multiply(u[:, 2::2], np.sin(T[:, 2::3])) - np.multiply(T[:, 0::3], u[:, 1, None])
        dT[:, 2::3] = u[:, 3::2] - u[:, 1, None]

        return dT

    def fwd_sim(self, dt, u):
        self.state = rk4(self.state.T, u, dt, self.dynamics).T
        return self.state

    def pfLagCompensation(self, state, u, dt):
        state2 = rk4(state.T, u, dt, self.dynamics).T
        return state2

    def pfStateTransition(self, prevParticles, dt, u):
        predictParticles = np.zeros(prevParticles.shape)

        N = prevParticles.shape[0]
        new_u = multivariate_normal(
            u, np.diag(dt*np.asarray(self.noise_u)), N)
        # np.asarray(u.T)[0], np.diag(dt*np.asarray(self.noise_u)), N))
        predictParticles = rk4(prevParticles, new_u, dt, self.dynamics)

        return predictParticles

    def pfMeasurementLikelihood(self, predictParticles, measurement, m1, m2):
        # n1 and n2 are their positions in the array
        if m1 < m2:
            n1 = m1
            n2 = m2
        else:
            n1 = m2
            n2 = m1

        xi = 3 * (n2 - 1)
        yi = 3 * (n2 - 1) + 1

        if n1 != 0:
            xj = 3 * (n1 - 1)
            yj = 3 * (n1 - 1) + 1
            dx = predictParticles[:, xi] - predictParticles[:, xj]
            dy = predictParticles[:, yi] - predictParticles[:, yj]
            d = np.sqrt(np.square(dx) + np.square(dy))
        else:
            d = np.sqrt(np.square(predictParticles[:, xi]) +
                        np.square(predictParticles[:, yi]))

        likelihood = norm.pdf(d, measurement, np.sqrt(self.noise_uwb))
        return likelihood[:, None]
