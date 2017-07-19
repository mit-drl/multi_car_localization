
import numpy as np
from scipy.stats import multivariate_normal
from scipy.stats import rv_discrete

"""
State:
    [x0, y0
     x1, y1
     x2, y2]

Control:
    [dx0, dy0
     dx1, dy1
     dx2, dy2]

Measurements:
    [gps_x0, gps_y0, uwb_00, uwb_01, uwb_02,
     gps_x1, gps_y1, uwb_10, uwb_11, uwb_12
     gps_x2, gps_y2, uwb_20, uwb_21, uwb_22]
"""


class MultiCarParticleFilter(object):

    def __init__(self, **kwargs):
        self.Np = kwargs.get("num_particles", 100)
        self.Ncars = kwargs.get("num_cars", 3)
        self.Ndim = kwargs.get("num_state_dim", 2)
        self.Nmeas = kwargs.get("num_measurements", 5)
        self.x0 = kwargs.get(
            "x0", np.array([[0, 0], [2, 1], [0, 1]]))
        self.x_cov = kwargs.get(
            "x_cov",
            np.diag(self.Ncars * self.Ndim * [0.1]))
        self.meas_cov = kwargs.get(
            "measurement_cov",
            np.diag(self.Ncars * [0.1, 0.1, 0.01, 0.01, 0.01]))
        self.control_cov = kwargs.get(
            "control_cov",
            np.diag(self.Ncars * self.Ndim * [0.1]))
        self.resample_perc = kwargs.get("resample_perc", 0.3)
        self.particles = np.random.multivariate_normal(
            self.x0.flatten(),
            self.x_cov,
            size=self.Np).reshape((self.Np, self.Ncars, self.Ndim))
        self.weights = 1.0 / self.Np * np.ones((self.Np,))
        self.control_probs = np.ones_like(self.weights)

    def pdf(self, meas, mean, cov):
        return multivariate_normal.pdf(
            meas.flatten(), mean=mean.flatten(), cov=cov)

    def sample(self, mean, cov):
        return np.random.multivariate_normal(
            mean.flatten(), cov).reshape(mean.shape)

    def predict(self, u, dt):
        for j in xrange(self.Np):
            u_noise = self.sample(np.zeros_like(self.x0), self.x_cov)
            new_particle = self.particles[j] + u * dt
            self.control_probs[j] = self.pdf(
                new_particle + u_noise,
                new_particle,
                self.x_cov)
            self.particles[j] = new_particle + u_noise
        return self

    def update_weights(self, meas):
        for j in xrange(self.Np):
            p_means = np.zeros((self.Ncars, self.Nmeas))
            for k in xrange(self.Ncars):
                p_means[k, :2] = self.particles[j, k, :2]
                for l in xrange(self.Ncars):
                    if k != l:
                        p_means[k, l + 2] = np.linalg.norm(
                            self.particles[j, k] - self.particles[j, l])
            self.weights[j] *= self.pdf(meas, p_means, self.meas_cov)
        self.weights += 1e-32
        self.weights /= self.weights.sum()
        return self

    def resample(self):
        n_eff = 1.0 / (self.weights ** 2).sum()
        if n_eff < self.resample_perc * self.Np:
            distr = rv_discrete(values=(np.arange(self.Np), self.weights))
            self.particles = self.particles[distr.rvs(size=self.Np)]
            self.weights = 1.0 / self.Np * np.ones_like(self.weights)
        return self

    def predicted_state(self):
        pred = np.zeros_like(self.x0)
        total_weight = self.weights.sum()
        for i in xrange(self.Np):
            pred += (self.weights[i] / total_weight) * self.particles[i]
        return pred
