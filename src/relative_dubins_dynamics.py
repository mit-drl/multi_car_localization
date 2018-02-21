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
        u1 = u[:2]
        numT = np.shape(T)[0] / 3
        dT = np.asmatrix(np.zeros(np.shape(T)))
        for i in range(numT):
            fi = 3 * i
            uj = u[2 * (i + 1):2 * (i + 1) + 2]
            dT[fi, 0]     = uj[0, 0] * np.cos(T[fi + 2, 0]) + T[fi + 1, 0] * u1[1, 0] - u1[0, 0]
            dT[fi + 1, 0] = uj[0, 0] * np.sin(T[fi + 2, 0]) - T[fi, 0] * u1[1, 0]
            dT[fi + 2, 0] = uj[1, 0] - u1[1, 0]

        return dT

    def dynamics_mat(self, T, u):
        # T: a N x d matrix
        # u: a N x m matrix

        dT = np.asmatrix(np.zeros(np.shape(T)))

        dT[:, 0::3] = np.multiply(u[:, 2::2], np.cos(T[:, 2::3])) + np.multiply(T[:, 1::3], u[:, 1]) - u[:, 0]
        dT[:, 1::3] = np.multiply(u[:, 2::2], np.sin(T[:, 2::3])) - np.multiply(T[:, 0::3], u[:, 1])
        dT[:, 2::3] = u[:, 3::2] - u[:, 1]

        return dT

    def fwd_sim(self, dt, u):
        self.state = rk4(self.state, u, dt, self.dynamics)
        return self.state

    def pfStateTransition(self, prevParticles, dt, u):
        predictParticles = np.asmatrix(np.zeros(np.shape(prevParticles)))

        N = np.shape(prevParticles)[0]

        new_u = np.asmatrix(multivariate_normal(
            np.asarray(u.T)[0],
            np.diag(self.noise_u), N))

        # state = prevParticles.T
        # ui = new_u.T
        # predictParticles = rk4(state, ui, dt, self.dynamics_mat).T

        predictParticles = rk4(prevParticles, new_u, dt, self.dynamics_mat)

        # for i in np.arange(N):
        #     state = prevParticles[i].T
        #     ui = new_u[i].T
        #     newParticle = rk4(state, ui, dt, self.dynamics)
        #     # newParticle = rk4(state, ui, dt, self.dynamics)
        #     predictParticles[i, :] = newParticle.T

        return predictParticles

    # def pfStateTransitionIndividual(self, prevParticles, dt, u, car_index):
    #     # car_index corresponds to the order of the cars in the state array:
    #     # car_index = 0 means it is the ego car and all transforms are affected
    #     # whereas car_index = 1 means it is the first car in the array and only
    #     # the first transform is affected, etc
    #     predictParticles = np.asmatrix(np.zeros(np.shape(prevParticles)))

    #     N = np.shape(prevParticles)[0]

    #     r = np.asmatrix(multivariate_normal(
    #         np.array([0]*(self.Ncars-1)*3),
    #         np.diag(self.noise_u), N))

    #     for i in range(self.Ncars):
    #         if i != car_index:
    #             r[:, 2*i] = 0
    #             r[:, 2*i+1] = 0

    #     full_u = np.asmatrix(np.zeros(((self.Ncars-1)*3, 1)))
    #     full_u[2*car_index:2*car_index+2, :] = u.T
    #     new_u = r + full_u.T

    #     for i in np.arange(np.shape(prevParticles)[0]):
    #         state = prevParticles[i].T
    #         u = new_u[i].T
    #         newParticle = rk4(state, u, dt, self.dynamics)
    #         predictParticles[i, :] = newParticle.T

    #     return predictParticles

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
