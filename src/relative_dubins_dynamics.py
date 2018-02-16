import numpy as np
from numpy.random import uniform
from numpy.random import randn
# For random samples from N(\mu, \sigma^2), use:
# sigma * np.random.randn(...) + mu
from scipy.stats import norm
from numpy.random import multivariate_normal
from utils import rk4
import pdb


class RelativeDubinsDynamics(object):

    def __init__(self, Ncars, initial_transforms, noise_u, noise_uwb):
        self.initial_transforms = initial_transforms
        self.Ncars = Ncars
        self.noise_u = noise_u
        self.noise_uwb = noise_uwb

    def dynamics(self, T, u):
        u1 = u[:2]
        numT = np.shape(T)[0] / 3
        dT = np.asmatrix(np.zeros(np.shape(T)))
        for i in range(numT):
            fi = 3 * i
            uj = u[2 * (i + 1):2 * (i + 1) + 2]
            dT[fi] = uj.item(0) * np.cos(T[fi + 2].item()) + \
                T[fi + 1].item() * u1.item(1) - u1.item(0)
            dT[fi + 1] = uj.item(0) * np.sin(T[fi + 2].item()
                                             ) - T[fi].item() * u1.item(1)
            dT[fi + 2] = uj.item(1) - u1.item(1)

        return dT

    def pfStateTransition(self, prevParticles, dt, u):
        predictParticles = np.asmatrix(np.zeros(np.shape(prevParticles)))

        N = np.shape(prevParticles)[0]

        r = np.asmatrix(multivariate_normal(
            (0, 0, 0, 0, 0, 0), np.diag([0.1, 0.05, 0.1, 0.05, 0.1, 0.05]), N))

        new_u = r + u.T

        for i in np.arange(np.shape(prevParticles)[0]):
            state = prevParticles[i].T
            u = new_u[i].T
            newParticle = rk4(state, u, dt, self.dynamics)
            predictParticles[i, :] = newParticle.T

        return predictParticles

    def pfMeasurementLikelihood(self, predictParticles, measurement, n1, n2):
        xi = 3 * (n2 - 2)
        yi = 3 * (n2 - 2) + 1

        if n1 != 1:
            xj = 3 * (n1 - 2)
            yj = 3 * (n1 - 2) + 1
            dx = predictParticles[:, xi] - predictParticles[:, xj]
            dy = predictParticles[:, yi] - predictParticles[:, yj]
            d = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        else:
            d = np.sqrt(np.power(predictParticles[:, xi], 2) + np.power(predictParticles[:, yi], 2))

        likelihood = norm.pdf(d, measurement, np.sqrt(self.noise_uwb))
        return likelihood
