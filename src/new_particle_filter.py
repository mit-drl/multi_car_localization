import numpy as np
from numpy.random import uniform
from numpy.random import randn
import time
# For random samples from N(\mu, \sigma^2), use:
# sigma * np.random.randn(...) + mu
from numpy.linalg import norm
from relative_dubins_dynamics import RelativeDubinsDynamics
from filterpy.monte_carlo import systematic_resample
import matplotlib.pyplot as plt
import pdb
import cProfile
import re


class ParticleFilter(object):

    def __init__(self):
        self.StateTransitionFcn = None
        self.MeasurementLikelihoodFcn = None
        self.LagCompensationFcn = None
        self.particles = None
        self.weights = None

    def rot(self, theta):
        return np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    # bounds is d x 2 where d is dimensions of a particle
    def create_uniform_particles(self, N, bounds, circ_var):
        dim = np.shape(bounds)[0]

        particles = np.empty((N, dim))

        for i in np.arange(dim):
            particles[:, i] = uniform(bounds[i, 0], bounds[i, 1], size=N).T
            if circ_var[i] == 1:
                particles[:, i] %= 2 * np.pi

        self.particles = particles
        self.weights = np.ones((N, 1))
        self.weights /= np.sum(self.weights)

    def correct(self, *args):
        self.weights = np.multiply(self.weights, self.MeasurementLikelihoodFcn(
            self.particles, *args))

        self.weights += 1.e-100     # avoid round-off to zero
        self.weights /= np.sum(self.weights)

        N = self.weights.shape[0]
        if self.neff(self.weights) < 0.7*N:
            print "RESAMPLED"
            indexes = systematic_resample(self.weights)
            self.particles, self.weights = self.resample_from_index(
                self.particles, self.weights, indexes)

        return self.estimate(self.particles, self.weights)

    # u should be a column matrix
    # args are in the order dt, u, etc
    def predict(self, *args):
        if self.StateTransitionFcn is None:
            raise Exception(
                "The particle filter has not been \
                 given a state transition function")

        self.particles = self.StateTransitionFcn(self.particles, *args)

    def lag_compensate(self, *args):
        if self.LagCompensationFcn is None:
            raise Exception(
                "You need a lag compensation function")

        return self.LagCompensationFcn(*args)

    def estimate(self, particles, weights):
        # top 10%
        N = particles.shape[0]
        top = int(N*0.2)
        ind = np.argpartition(weights[:, 0], -top)[-top:]

        mean = np.average(
            particles[ind], weights=weights[ind].T[0], axis=0).T
        var = np.average(np.power((particles[ind] - mean.T), 2),
                         weights=weights[ind].T[0], axis=0)
        theta_x = np.average(np.cos(particles[ind,2::3]), weights=weights[ind].T[0],axis=0).T
        theta_y = np.average(np.sin(particles[ind,2::3]), weights=weights[ind].T[0],axis=0).T
        theta = np.arctan2(theta_y, theta_x)
        mean[2::3] = theta
        return mean, var

    def get_state(self):
        mean, var = self.estimate(self.particles, self.weights)
        return mean, var

    def get_trace(self):
        traces = []
        N = self.particles.shape[1]/3
        for i in range(N):
            fi = 3*i
            # var = np.average(np.power((self.particles - mean.T), 2),
            #                  weights=self.weights.T[0], axis=0)
            trace = np.trace(np.cov(self.particles[:, fi:fi+3].T,
                             aweights=self.weights.T[0]))
            traces.append(trace)
        return traces

    def resample_from_index(self, particles, weights, indexes):
        particles[:] = particles[indexes]
        weights[:] = weights[indexes]
        weights.fill(1.0 / len(weights))

        return particles, weights

    def multinomial_resample(self, particles, weights):
        N = len(particles)
        cumulative_sum = np.cumsum(weights)
        cumulative_sum[-1] = 1.
        indexes = np.searchsorted(cumulative_sum, uniform(0, 1, N))

        particles[:] = particles[indexes]
        weights.fill(1.0 / N)
        return particles, weights

    def neff(self, weights):
        neff = 1. / np.sum(np.square(weights))
        return neff


if __name__ == "__main__":
    pf = ParticleFilter()

    Ncars = 3
    Nparticles = 1000
    total_time = 5.
    dt = 0.03

    initial_positions = np.array(
        [[0, 0, np.pi / 4], [2, 3, -np.pi / 4], [-1, 2, np.pi]])
    initial_transforms = np.zeros(((Ncars - 1), 3))
    circ_var = [0, 0, 1, 0, 0, 1]
    limits = np.array([0.5, 0.5, np.pi / 2, 0.5, 0.5, np.pi / 2]).T
    bounds = np.zeros((3 * (Ncars - 1), 2))

    for i in range(Ncars - 1):
        init_xy = np.dot(pf.rot(-initial_positions[0, 2]),
                         initial_positions[i + 1, :2])
        init_theta = initial_positions[i + 1, 2] - initial_positions[0, 2]
        initial_transforms[i, :] = np.append(init_xy, init_theta)

        fi = 3 * i
        bounds[fi:fi + 3, 0] = initial_transforms[i, :].T - limits[fi:fi + 3]
        bounds[fi:fi + 3, 1] = initial_transforms[i, :].T + limits[fi:fi + 3]

    noise_u = [1.2, 1.2] * Ncars
    noise_uwb = 0.3

    rel_model = RelativeDubinsDynamics(
        Ncars, initial_transforms, noise_u, noise_uwb)

    pf.StateTransitionFcn = rel_model.pfStateTransition
    pf.MeasurementLikelihoodFcn = rel_model.pfMeasurementLikelihood
    pf.create_uniform_particles(Nparticles, bounds, circ_var)

    plt.figure()
    plt.scatter([pf.particles[:, 0]], [pf.particles[:, 1]],
                color='k', marker=',', s=1)
    plt.scatter([pf.particles[:, 3]], [pf.particles[:, 4]],
                color='g', marker=',', s=1)

    prev_t = 0
    u = [0.0] * Ncars * 2
    meas_u = np.array([0.0] * Ncars * 2)
    for t in np.arange(dt, total_time, dt):
        start = time.time()

        u[0::2] = [0.7*abs(np.sin(t)) + 0.1] * Ncars
        u[0] = 0.
        u[1] = 0.  # np.cos(2*t)
        u[3::2] = [0.]*(Ncars-1)  # [-0.12*np.cos(t)] * (Ncars - 1)

        rel_model.fwd_sim(dt, np.asarray(u))
        if uniform() < 0.95:
            delt = t - prev_t # + randn() * 0.003
            for i in range(Ncars):
                if uniform() < 0.95:
                    meas_u[2*i:2*i+2] = np.asarray(u[2*i:2*i+2]) + \
                                        np.multiply(np.sqrt(delt * np.asarray(noise_u[2*i:2*i+2])), randn(2))

            # DELETE THIS LATTERRRRRRRRRRRRRRRRRRRRRRRR
            meas_u = np.array(u)
            pf.predict(delt, meas_u)
            prev_t = t

        for i in np.arange(Ncars - 1):
            for j in np.arange(i + 1, Ncars):
                if i == 0:
                    fi = 3 * (j - 1)
                    measurement = norm(rel_model.state[fi:fi + 2]) + \
                        np.sqrt(noise_uwb) * randn(1)[0]
                else:
                    fi = 3 * (i - 1)
                    si = 3 * (j - 1)
                    fx = rel_model.state[fi:fi + 2]
                    sx = rel_model.state[si:si + 2]
                    measurement = norm(fx - sx) + \
                        np.sqrt(noise_uwb) * randn(1)[0]
                if uniform() < 0.93:
                    stateCorrected = pf.correct(
                            measurement, i, j)
        end_time = time.time()
        # print end_time - start

        # pdb.set_trace()
        state, var = pf.get_state()
        print var

        plt.scatter([state[0], state[3]],
                    [state[1], state[4]],
                    marker='+', color='r')
        true_state = rel_model.state
        plt.scatter([true_state[0], true_state[3]],
                    [true_state[1], true_state[4]],
                    marker='o', color='b')
        # plt.scatter([pf.particles[:, 0]], [pf.particles[:, 1]],
        #             color='k', marker=',', s=1)
        # plt.scatter([pf.particles[:, 3]], [pf.particles[:, 4]],
        #             color='g', marker=',', s=1)

    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.show()
