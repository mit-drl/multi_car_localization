
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.stats import rv_discrete
from scipy.spatial import distance
import rospy
import dynamics

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
    [gps_x0, gps_y0, lid_x0, lid_y0, lid_theta0, uwb_00, uwb_01, uwb_02
     gps_x1, gps_y1, lid_x1, lid_y1, lid_theta1, uwb_10, uwb_11, uwb_12
     gps_x2, gps_y2, lid_x2, lid_y2, lid_theta2, uwb_20, uwb_21, uwb_22]
"""


class MultiCarParticleFilter(object):

    def __init__(self, **kwargs):
        self.Np = kwargs.get("num_particles", 100)
        self.Ncars = kwargs.get("num_cars", 3)
        self.dynamics_model = kwargs.get("dynamics_model", "dubins")
        self.robot = dynamics.model(self.dynamics_model)

        self.Ndim = self.robot.Ndim
        self.Nmeas = kwargs.get("num_measurements", 6)
        self.x0 = kwargs.get("x0")
        self.x_cov = kwargs.get(
            "x_cov")
        self.init_cov = kwargs.get(
            "init_cov")
        self.meas_cov = kwargs.get(
            "measurement_cov")
        self.control_cov = kwargs.get(
            "control_cov",
            np.diag(self.Ncars * self.Ndim * [0.1]))
        self.resample_perc = kwargs.get("resample_perc", 0.3)
        self.particles = np.random.multivariate_normal(
            self.x0.flatten(),
            self.init_cov,
            size=self.Np).reshape((self.Np, self.Ncars, self.Ndim))
        self.weights = 1.0 / self.Np * np.ones((self.Np,))
        self.prev_angle_estimate = 0

    def pdf(self, meas, mean, cov):
        return multivariate_normal.pdf(
            meas.reshape(meas.shape[0], mean.size), mean=mean.flatten(), cov=cov)

    def sample(self, mean, cov, size):
        return np.random.multivariate_normal(
            mean.flatten(), cov, size).reshape(size + mean.shape)

    def update_particles(self, u, dt):
        u_noise = self.sample(np.zeros_like(self.x0), self.x_cov, (self.Np,))
        # new_particles = np.zeros((self.Np,) + self.x0.shape)
        # new_particle = self.particles[j] + u * dt
        new_particles = self.robot.state_transition(self.particles, u, dt)
        # self.weights *= self.pdf(new_particles + u_noise, new_particles, self.x_cov)
        self.particles = new_particles + u_noise
        return self.particles

    def set_meas_cov(self, new_meas_cov):
        self.meas_cov = new_meas_cov

    def update_weights(self, meas):
        #print self.weights
        #avg_error = np.zeros_like(meas)
        p_means = np.zeros((self.Np, self.Ncars, self.Nmeas))
        # gps measurements
        p_means[:, :, :2] = self.particles[:, :, :2]
        p_means[:, :, 2:5] = self.particles[:, :]
        for j in xrange(self.Np):
            # uwb measurements
            # compute all-pairs distances
            p_means[j, :, 5:] = distance.squareform(distance.pdist(self.particles[j, :, :2]))
            #avg_error += np.abs(meas - p_means) / self.Np
        self.weights *= self.pdf(p_means, meas, self.meas_cov)
        #self.weights += 1e-32
        #print avg_error
        #print self.weights.sum()
        if self.weights.sum() != 0:
            self.weights /= self.weights.sum()
            # print "after: %f" % (self.weights.sum())
        else:
            self.weights = np.ones_like(self.weights) / self.Np
            print "NEEDED TO RESAMPLEEE"
            self.resample()
        return self

    def resample(self):
        n_eff = 1.0 / (self.weights ** 2).sum()
        #rospy.loginfo("n_eff: %f" % (n_eff))
        if n_eff < self.resample_perc * self.Np:
            distr = rv_discrete(values=(np.arange(self.Np), self.weights))
            self.particles = self.particles[distr.rvs(size=self.Np)]
            u_noise = self.sample(np.zeros_like(self.x0), self.x_cov,
                                  self.particles.shape[:1])
            self.particles += u_noise
            self.weights = np.ones_like(self.weights) / self.Np
        return self

    def predicted_state(self):
        pred = np.zeros_like(self.x0)
        a_x = 0
        a_y = 0
        #total_weight = self.weights.sum()
        top_weight_indices = np.flipud(np.argsort(self.weights))[:10]
        total_weight = self.weights[top_weight_indices].sum()
        pred[2] -= self.prev_angle_estimate
        for i in top_weight_indices:
            #a_x += (self.weights[i] / total_weight)*np.cos(self.particles[i, 2])
            #a_y += (self.weights[i] / total_weight)*np.sin(self.particles[i, 2])
            pred += (self.weights[i] / total_weight) * self.particles[i]
        #angle = np.arctan2(a_y, a_x)
        #pred[2] = angle
        pred[2] += self.prev_angle_estimate
        self.prev_angle_estimate = pred[2]
        return pred

if __name__ == "__main__":
    from tqdm import trange

    Np = 150
    Ncars = 3
    Ndim = 3
    Nmeas = 5
    dt = 0.1
    x0 = np.array([[0, 0, math.pi],
                   [2, 1, math.pi/3.],
                   [0, 1, 0]],
                  dtype=np.float64)
    init_cov = np.diag(Ncars * [0.6, 0.6, 0.1])
    x_cov = np.diag(Ncars * [0.1, 0.1, 0.1])
    measurement_cov = np.diag(Ncars * [0.6, 0.6, 0.1, 0.1, 0.1])
    actual_meas_cov = measurement_cov
    # actual_meas_cov = np.diag(Ncars * [0.001, 0.001, 0.01, 0.01, 0.01])
    u_func = lambda t: np.array([
        [0.5, 7.0],
        [-0.2, 7.0],
        [0.03, 7.0]])
    # u_func = lambda t: np.array([
    #     [0.5, 0.0],
    #     [-0.2, 0.0],
    #     [0.03, 0.0]])
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

        xs[i] = mcpf.robot.state_transition(xs[i - 1], us, dt)
        #xs[i] += mcpf.sample(np.zeros_like(x0), x_cov)

        means = np.zeros((Ncars, Nmeas))

        for j in xrange(Ncars):
            # gps
            means[j, :2] = xs[i, j, :2]
            # velocity
            #means[j, 6] = xs[i, j, 4]
            # steering angle
            # velocity
            #means[j, 5] = xs[i, j, 3]
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
