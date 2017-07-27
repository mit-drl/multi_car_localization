
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.stats import rv_discrete

"""
State:
    [x0, y0, phi (angle), delta (steering angle), v
     x1, y1
     x2, y2]

Control:
    [dx0, dy0
     dx1, dy1
     dx2, dy2]

Measurements:
    [gps_x0, gps_y0, uwb_00, uwb_01, uwb_02, steerangle0, vel0
     gps_x1, gps_y1, uwb_10, uwb_11, uwb_12, steerangle1, vel1
     gps_x2, gps_y2, uwb_20, uwb_21, uwb_22, steerangle2, vel2]
"""


class MultiCarParticleFilter(object):

    def __init__(self, **kwargs):
        self.Np = kwargs.get("num_particles", 100)
        self.Ncars = kwargs.get("num_cars", 3)
        self.Ndim = kwargs.get("num_state_dim", 4)
        self.Nmeas = kwargs.get("num_measurements", 6)
        self.x0 = kwargs.get(
            "x0", np.array([[0, 0], [2, 1], [0, 1]]))
        self.x_cov = kwargs.get(
            "x_cov",
            np.diag(self.Ncars * self.Ndim * [0.1]))
        self.init_cov = kwargs.get(
            "init_cov",
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
            self.init_cov,
            size=self.Np).reshape((self.Np, self.Ncars, self.Ndim))
        self.weights = 1.0 / self.Np * np.ones((self.Np,))

    def pdf(self, meas, mean, cov):
        return multivariate_normal.pdf(
            meas.flatten(), mean=mean.flatten(), cov=cov)

    def sample(self, mean, cov):
        return np.random.multivariate_normal(
            mean.flatten(), cov).reshape(mean.shape)

    def state_transition(self, x, u, dt):
        # u is a tuple (u_d, u_a)
        steps = 1.
        h = dt/steps
        x = [x]
        for i in range(0, int(steps)):
            k1 = self.state_transition_model(x[i], u)
            k2 = self.state_transition_model(x[i] + 0.5*h*k1, u)
            k3 = self.state_transition_model(x[i] + 0.5*h*k2, u)
            k4 = self.state_transition_model(x[i] + k3*h, u)
            new_x = x[i] + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
            x.append(new_x)
        return x[-1]

    def state_transition_model(self, state, u):
        u_d, u_a, = u
        x, y, phi, v = state
        dx = v*math.cos(phi)
        dy = v*math.sin(phi)
        dphi = (v/3.)*math.tan(u_d)
        dv = u_a
        return np.array([dx, dy, dphi, dv])

    # x0 = np.array([0, 0, math.pi/2.0, 0.2, 1.0])
    # dt = 3.0
    # u = (-0.2, 1.0)
    # steps = 100
    #
    # states = erk4(f, x0, dt, u, steps)

    def update_particles(self, u, dt):
        for i in xrange(self.Np):
            u_noise = self.sample(np.zeros_like(self.x0), self.x_cov)
            # new_particle = self.particles[j] + u * dt
            # self.particles[j] = new_particle + u_noise
            for j in xrange(self.Ncars):
                self.particles[i, j] = self.state_transition(
                    self.particles[i, j], u[j], dt)
            self.particles[i] += u_noise
        return self

    def update_weights(self, meas):
        for j in xrange(self.Np):
            p_means = np.zeros((self.Ncars, self.Nmeas))
            for k in xrange(self.Ncars):
                # 'gps measurements'
                p_means[k, :2] = self.particles[j, k, :2]
                # p_means[k, 6] = self.particles[j, k, 4]
                #p_means[k, 5] = self.particles[j, k, 3]
                # 'uwb measurements'
                for l in xrange(self.Ncars):
                    if k != l:
                        p_means[k, l + 2] = np.linalg.norm(
                            self.particles[j, k, :2] - self.particles[j, l, :2])
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

    Np = 150
    Ncars = 3
    Ndim = 4
    Nmeas = 6
    dt = 0.1
    x0 = np.array([[0, 0, math.pi, 0],
                   [2, 1, math.pi/3., 0],
                   [0, 1, 0, 0]],
                  dtype=np.float64)
    init_cov = np.diag(Ncars * [0.6, 0.6, 0.1, 0.00])
    x_cov = np.diag(Ncars * [0.1, 0.1, 0.1, 0.0])
    measurement_cov = np.diag(Ncars * [0.6, 0.6, 0.1, 0.1, 0.1, 0.01])
    actual_meas_cov = measurement_cov
    # actual_meas_cov = np.diag(Ncars * [0.001, 0.001, 0.01, 0.01, 0.01])
    u_func = lambda t: np.array([
        [0.1, 0.1],
        [-0.2, 0.1],
        [0.03, 0.1]])
    Nsecs = 20.0
    Nsteps = int(Nsecs / dt)
    xs = np.zeros((Nsteps, Ncars, Ndim))
    measurements = np.zeros((Nsteps - 1, Ncars, Nmeas))
    xs_pred = np.zeros_like(xs)
    xs[0] = x0
    xs_pred[0] = x0

    mcpf = MultiCarParticleFilter(
        num_particles=Np,
        num_cars=Ncars,
        num_state_dim=Ndim,
        num_measurements=Nmeas,
        x0=x0,
        init_cov=init_cov,
        x_cov=x_cov,
        measurement_cov=measurement_cov,
        resample_perc=0.3)

    error = np.zeros((Nsteps,))

    for i in trange(1, Nsteps):
        us = u_func(i * dt)

        for j in xrange(Ncars):
            xs[i, j] = mcpf.state_transition(xs[i - 1, j], us[j], dt)
            #xs[i] += mcpf.sample(np.zeros_like(x0), x_cov)

        means = np.zeros((Ncars, Nmeas))

        for j in xrange(Ncars):
            # gps
            means[j, :2] = xs[i, j, :2]
            # velocity
            #means[j, 6] = xs[i, j, 4]
            # steering angle
            # velocity
            means[j, 5] = xs[i, j, 3]
            # uwbs
            for k in xrange(Ncars):
                if j != k:
                    means[j, k + 2] = np.linalg.norm(xs[i, j, :2] - xs[i, k, :2])
        meas = np.random.multivariate_normal(
            means.flatten(), actual_meas_cov).reshape(Ncars, Nmeas)
        measurements[i - 1] = meas

        mcpf.update_particles(us, dt)
        mcpf.update_weights(meas)
        xs_pred[i] = mcpf.predicted_state()
        mcpf.resample()
        for j in xrange(Ncars):
            error[i] += np.linalg.norm(xs_pred[i, j, :2] - xs[i, j, :2]) / Ncars

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    axes[0].plot(xs[:, 0, 0], xs[:, 0, 1], "r")
    axes[0].plot(xs[:, 1, 0], xs[:, 1, 1], "g")
    axes[0].plot(xs[:, 2, 0], xs[:, 2, 1], "b")

    axes[0].scatter(xs_pred[:, 0, 0], xs_pred[:, 0, 1], c="r")
    axes[0].scatter(xs_pred[:, 1, 0], xs_pred[:, 1, 1], c="g")
    axes[0].scatter(xs_pred[:, 2, 0], xs_pred[:, 2, 1], c="b")

    # axes[0].scatter(measurements[:, 0, 0], measurements[:, 0, 1], c="r", marker="*")
    # axes[0].scatter(measurements[:, 1, 0], measurements[:, 1, 1], c="g", marker="*")
    # axes[0].scatter(measurements[:, 2, 0], measurements[:, 2, 1], c="b", marker="*")

    axes[1].plot(error)
    plt.show()
