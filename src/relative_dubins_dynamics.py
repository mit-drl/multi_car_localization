import numpy as np
from scipy.stats import norm
from numpy.random import multivariate_normal
from utils import rk4
from utils import newton
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

        dT = np.asmatrix(np.zeros(np.shape(T)))

        dT[:, 0::3] = np.multiply(u[:, 2::2], np.cos(T[:, 2::3])) + np.multiply(T[:, 1::3], u[:, 1]) - u[:, 0]
        dT[:, 1::3] = np.multiply(u[:, 2::2], np.sin(T[:, 2::3])) - np.multiply(T[:, 0::3], u[:, 1])
        dT[:, 2::3] = u[:, 3::2] - u[:, 1]

        return dT

    def fwd_sim(self, dt, u):
        self.state = rk4(self.state.T, u.T, dt, self.dynamics).T
        return self.state

    def pfStateTransition(self, prevParticles, dt, u):
        predictParticles = np.asmatrix(np.zeros(np.shape(prevParticles)))

        N = np.shape(prevParticles)[0]

        new_u = np.asmatrix(multivariate_normal(
            np.asarray(u.T)[0], np.diag(dt*np.asarray(self.noise_u)), N))
        # np.asarray(u.T)[0], np.diag(dt*np.asarray(self.noise_u)), N))

        predictParticles = rk4(prevParticles, new_u, dt, self.dynamics)

        return predictParticles

    def pfMeasurementLikelihood(self, predictParticles, measurement, n1, n2):
        # n1 and n2 are the car ids, not their positions in an array
        xi = 3 * (n2 - 2)
        yi = 3 * (n2 - 2) + 1

        if n1 != 1:
            xj = 3 * (n1 - 2)
            yj = 3 * (n1 - 2) + 1
            dx = predictParticles[:, xi] - predictParticles[:, xj]
            dy = predictParticles[:, yi] - predictParticles[:, yj]
            d = np.sqrt(np.square(dx) + np.square(dy))
        else:
            d = np.sqrt(np.square(predictParticles[:, xi]) +
                        np.square(predictParticles[:, yi]))

        likelihood = norm.pdf(d, measurement, np.sqrt(self.noise_uwb))
        return likelihood
