import numpy as np
from numpy.random import uniform
from numpy.random import randn
# For random samples from N(\mu, \sigma^2), use:
# sigma * np.random.randn(...) + mu
from numpy.linalg import norm
from relative_dubins_dynamics import RelativeDubinsDynamics
from filterpy.monte_carlo import systematic_resample
import matplotlib.pyplot as plt
import pdb


class ParticleFilter(object):

    def __init__(self):
        self.StateTransitionFcn = None
        self.MeasurementLikelihoodFcn = None
        self.particles = None
        self.weights = None

    def rot(self, theta):
        return np.matrix(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    # bounds is d x 2 where d is dimensions of a particle
    def create_uniform_particles(self, N, bounds, circ_var):
        dim = np.shape(bounds)[0]

        particles = np.asmatrix(np.empty((N, dim)))

        for i in np.arange(dim):
            particles[:, i] = np.asmatrix(
                uniform(bounds[i, 0], bounds[i, 1], size=N)).T
            if circ_var[i] == 1:
                particles[:, i] %= 2 * np.pi

        self.particles = particles
        self.weights = np.asmatrix(np.ones((N, 1)))
        self.weights /= sum(self.weights)

    def correct(self, *args):
        self.weights = np.multiply(self.weights, self.MeasurementLikelihoodFcn(
            self.particles, *args))

        self.weights += 1.e-100     # avoid round-off to zero
        self.weights /= sum(self.weights)

        N = np.shape(self.weights)[0]
        if self.neff(self.weights) < N/2:
            indexes = systematic_resample(np.asarray(self.weights.T)[0])
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

    def estimate(self, particles, weights):
        mean = np.average(
            particles, weights=np.asarray(weights.T)[0], axis=0).T
        var = np.average(np.power((particles - mean.T), 2),
                         weights=np.asarray(weights.T)[0], axis=0)
        return mean, var

    def get_state(self):
        mean, var = self.estimate(self.particles, self.weights)
        return mean

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
    Nparticles = 100
    total_time = 5.
    dt = 0.05

    initial_positions = np.matrix(
        [[0, 0, np.pi / 2], [1, 1, -np.pi / 2], [-1, -2, 0]])
    initial_transforms = np.asmatrix(np.zeros(((Ncars - 1), 3)))
    circ_var = [0, 0, 1, 0, 0, 1]
    limits = np.matrix([0.5, 0.5, np.pi / 2, 0.5, 0.5, np.pi / 2]).T
    bounds = np.asmatrix(np.zeros((3 * (Ncars - 1), 2)))

    for i in range(Ncars - 1):
        init_xy = pf.rot(-initial_positions[0, 2]) * \
            initial_positions[i + 1, :2].T
        init_theta = initial_positions[i + 1, 2] - initial_positions[0, 2]
        initial_transforms[i, :] = np.append(init_xy.T, [[init_theta]], axis=1)

        fi = 3 * i
        bounds[fi:fi + 3, 0] = initial_transforms[i, :].T - limits[fi:fi + 3]
        bounds[fi:fi + 3, 1] = initial_transforms[i, :].T + limits[fi:fi + 3]

    noise_u = np.matrix([0.1, 0.05, 0.1, 0.05, 0.1, 0.05])
    noise_uwb = 0.05

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

    for t in np.arange(0, total_time, dt):
        pf.predict(dt, np.matrix([1, 1, 1, 1, 1, 1]).T)
        rel_model.fwd_sim(dt, np.matrix([1, 1, 1, 1, 1, 1]).T)

        for i in np.arange(Ncars - 1):
            for j in np.arange(i + 1, Ncars):
                if i == 0:
                    fi = 3 * (j - 1)
                    measurement = norm(rel_model.state[0, fi:fi + 1]) + \
                        np.sqrt(noise_uwb) * randn(1)[0]
                else:
                    fi = 3 * (i - 1) + 1
                    si = 3 * (j - 1) + 1
                    fx = rel_model.state[i, fi:fi + 1]
                    sx = rel_model.state[j, si:si + 1]
                    measurement = norm(fx - sx) + \
                        np.sqrt(noise_uwb) * randn(1)[0]
                stateCorrected, covCorrected = pf.correct(
                        measurement, i, j)

        # pdb.set_trace()
        plt.scatter([stateCorrected[0], stateCorrected[3]],
                    [stateCorrected[1], stateCorrected[4]],
                    marker='+', color='r')
        plt.scatter([pf.particles[:, 0]], [pf.particles[:, 1]],
                    color='k', marker=',', s=1)
        plt.scatter([pf.particles[:, 3]], [pf.particles[:, 4]],
                    color='g', marker=',', s=1)

    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.show()
