
import numpy as np
import matplotlib.pyplot as plt
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


if __name__ == "__main__":
    from tqdm import trange

    Np = 100
    Ncars = 3
    Ndim = 2
    Nmeas = 5
    dt = 0.1
    x0 = np.array([[0, 0],
                [2, 1],
                [0, 1]], dtype=np.float64)
    x_cov = np.diag(Ncars * Ndim * [0.01])
    measurement_cov = np.diag(Ncars * [0.1, 0.1, 0.01, 0.01, 0.01])
    v_cov = np.diag(Ncars * Ndim * [0.1])
    v_func = lambda t: np.array([
        [0.1 * t, 3 * np.cos(t)],
        [2 * np.cos(0.5 * t), 1 * np.sin(t)],
        [1.4 * np.cos(t), 3 * np.sin(0.5 * t)]])
    Nsecs = 10.0
    Nsteps = int(Nsecs / dt)
    xs = np.zeros((Nsteps, Ncars, Ndim))
    xs_pred = np.zeros_like(xs)
    xs[0] = x0
    xs_pred[0] = x0

    mcpf = MultiCarParticleFilter(
        num_particles=Np,
        num_cars=Ncars,
        num_state_dim=Ndim,
        x0=x0,
        x_cov=x_cov,
        measurement_cov=measurement_cov,
        resample_perc=0.3)

    error = np.zeros((Nsteps,))

    for i in trange(1, Nsteps):
        vs_actual = v_func(i * dt)
        xs[i] = xs[i - 1] + vs_actual * dt
        vs = np.random.multivariate_normal(
            vs_actual.flatten(), v_cov).reshape(Ncars, Ndim)

        means = np.zeros((Ncars, Nmeas))
        for j in xrange(Ncars):
            means[j, :2] = xs[i, j, :2]
            for k in xrange(Ncars):
                if j != k:
                    means[j, k + 2] = np.linalg.norm(xs[i, j] - xs[i, k])
        meas = np.random.multivariate_normal(
            means.flatten(), measurement_cov).reshape(Ncars, Nmeas)
        mcpf.predict(vs, dt)
        mcpf.update_weights(meas)
        xs_pred[i] = mcpf.predicted_state()
        mcpf.resample()
        for j in xrange(Ncars):
            error[i] += np.linalg.norm(xs_pred[i, j] - xs[i, j]) / Ncars

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    axes[0].plot(xs[:, 0, 0], xs[:, 0, 1], "r")
    axes[0].plot(xs[:, 1, 0], xs[:, 1, 1], "g")
    axes[0].plot(xs[:, 2, 0], xs[:, 2, 1], "b")

    axes[0].scatter(xs_pred[:, 0, 0], xs_pred[:, 0, 1], c="r")
    axes[0].scatter(xs_pred[:, 1, 0], xs_pred[:, 1, 1], c="g")
    axes[0].scatter(xs_pred[:, 2, 0], xs_pred[:, 2, 1], c="b")

    axes[1].plot(error)
    plt.show()
