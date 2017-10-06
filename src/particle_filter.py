
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal, norm, rv_discrete
import rospy

import dynamics
import utils

"""
State:
    [x0, y0, phi (angle)]

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
        self.Np = kwargs.get('num_particles', 100)
        self.num_cars = kwargs['num_cars']
        self.connections = kwargs['connections']
        self.car_id = kwargs['car_id']
        self.car_index = self.connections.index(self.car_id)
        self.x0 = kwargs['x0']
        self.x_cov = kwargs['x_cov']
        self.uwb0 = kwargs['uwb0']
        self.pose_cov = kwargs['pose_cov']
        self.uwb_var = kwargs['uwb_var']
        self.mcpfs = {}
        self.id_to_index = {}
        for i, target in enumerate(self.connections):
            self.id_to_index[target] = i
            if target == self.car_id:
                self.mcpfs[target] = IdentityCarParticleFilter(
                    num_particles=self.Np,
                )
            else:
                self.mcpfs[target] = SingleCarParticleFilter(
                    num_particles=self.Np,
                    x_cov=self.x_cov,
                    local_pose=self.x0[i],
                    local_pose_cov=self.pose_cov,
                    uwb_meas=self.uwb0[(self.car_id, target)].distance,
                    uwb_var=self.uwb_var,
                    target_pose=self.x0[i],
                    target_pose_cov=self.pose_cov,
                )

    def update_particles(self, pose_meas):
        particles = []
        for i, (target, mcpf) in enumerate(sorted(self.mcpfs.items())):
            particles.append(mcpf.update_particles(pose_meas[i]))
        return np.stack(particles, axis=1)

    def update_weights(self, pose_meas, uwb_meas):
        poses = []
        covs = []
        for i, pose in enumerate(pose_meas):
            if pose is None:
                poses.append(None)
                covs.append(None)
            else:
                mcpf = self.mcpfs[self.connections[i]]
                pose, cov = utils.average(mcpf.transform(pose), mcpf.weights)
                poses.append(pose)
                covs.append(cov)
        for (car_t, car_s), uwb in uwb_meas.items():
            if uwb.distance > 0:
                ti = self.id_to_index[car_t]
                si = self.id_to_index[car_s]
                local_pose = poses[si]
                rel_pose = pose_meas[ti]
                if local_pose is None or rel_pose is None:
                    continue
                local_pose_cov = covs[si] + self.pose_cov
                rel_pose_cov = self.pose_cov
                self.mcpfs[car_t].update_weights(
                    # cheating by giving correct transformed pose (correct when transformation is identity)
                    # local_pose, local_pose_cov,
                    pose_meas[si], local_pose_cov,
                    uwb.distance, self.uwb_var,
                    rel_pose, rel_pose_cov)

    def resample(self):
        for mcpf in self.mcpfs.values():
            mcpf.resample()

    def predicted_state(self):
        states, covs = zip(*(mcpf.predicted_state() for i, mcpf in sorted(self.mcpfs.items())))
        return np.array(states), np.array(covs)

class SingleCarParticleFilter(object):

    def __init__(self, **kwargs):
        self.Np = kwargs['num_particles']
        self.x_cov = kwargs['x_cov']
        self.particles = self.init_particles(
            local_pose=kwargs['local_pose'],
            local_pose_cov=kwargs['local_pose_cov'],
            d=kwargs['uwb_meas'],
            d_var=kwargs['uwb_var'],
            rel_pose=kwargs['target_pose'],
            rel_pose_cov=kwargs['target_pose_cov'],
        )
        self.weights = np.ones(self.Np) / self.Np

    def init_particles(self,
                       local_pose, local_pose_cov,
                       d, d_var,
                       rel_pose, rel_pose_cov):
        noisy_local = np.random.multivariate_normal(local_pose, local_pose_cov, size=self.Np)
        distances = np.random.normal(d, math.sqrt(d_var), size=(self.Np,1))
        angles = np.random.uniform(0, 2*math.pi, size=(self.Np,1))
        rel = noisy_local[...,:2] + distances * np.concatenate((np.cos(angles), np.sin(angles)), axis=-1)
        thetas = np.random.uniform(0, 2*math.pi, size=(self.Np,1))
        noisy_rel = np.random.multivariate_normal(rel_pose, rel_pose_cov, size=self.Np)
        rel_rotated = utils.rotate(noisy_rel, thetas)
        offsets = rel - rel_rotated[...,:2]
        return np.concatenate((offsets, thetas), axis=-1)

    def itransform(self, pose):
        xys, thetas = self.particles[:,:2], particles[:,2:]
        poses = np.full_like(self.particles, pose)
        poses[...,:2] -= xys
        poses = utils.rotate(poses, -thetas)
        return poses

    def transform(self, pose):
        xys, thetas = self.particles[:,:2], self.particles[:,2:]
        poses = utils.rotate(pose, thetas)
        poses[...,:2] += xys
        return poses

    def update_particles(self, pose):
        if pose is not None:
            u_noise = np.random.multivariate_normal(np.zeros(3), self.x_cov, (self.Np,))
            # new_particles = self.robot.state_transition(self.particles, u, dt)
            origin = self.transform(pose)
            self.particles[...,:2] -= origin[...,:2]
            self.particles = utils.rotate(self.particles, u_noise[...,2:])
            self.particles[...,:2] += u_noise[...,:2]
            self.particles[...,:2] += origin[...,:2]
        return self.particles

    def update_weights(self,
                       local_pose, local_pose_cov,
                       d, d_var,
                       rel_pose, rel_pose_cov):
        # local_pose is the target pose in the ego car's frame
        # rel_pose is the source pose in the mcpf's frame
        # d is the distance between them
        rel_poses = self.transform(rel_pose)
        differences = rel_poses - local_pose
        local_pose_var = utils.directional_variance(local_pose_cov, differences)
        rel_pose_var = utils.directional_variance(rel_pose_cov, differences)
        var = local_pose_var + d_var + rel_pose_var
        distances = np.linalg.norm(differences[...,:2], axis=-1)
        FUDGE_NOISE = 1
        factors = norm.pdf(distances, d, np.sqrt(var)+FUDGE_NOISE)
        SMOOTH_FACTOR = 0.1
        self.weights *= norm.pdf(distances, d, np.sqrt(var))**SMOOTH_FACTOR

        if self.weights.sum() <= 0:
            self.weights = np.ones_like(self.weights) / self.Np
            print "NEEDED TO RESAMPLEEE"
            self.resample()

        self.weights /= self.weights.sum()

    def resample(self):
        resampled_indices = np.random.choice(np.arange(self.Np), self.Np, p=self.weights)
        self.particles = self.particles[resampled_indices]
        self.weights = np.ones_like(self.weights) / self.Np

    def predicted_state(self):
        return utils.average(self.particles, self.weights)

class IdentityCarParticleFilter(object):
    '''Identity transform'''

    def __init__(self, **kwargs):
        self.Np = kwargs['num_particles']
        self.particles = np.zeros((self.Np,3))
        self.weights = np.ones(self.Np) / self.Np

    def itransform(self, pose):
        return np.full_like(self.particles, pose)

    def transform(self, pose):
        return np.full_like(self.particles, pose)

    def update_particles(self, pose):
        return self.particles

    def update_weights(self,
                       local_pose, local_pose_cov,
                       d, d_var,
                       rel_pose, rel_pose_cov):
        pass

    def resample(self):
        pass

    def predicted_state(self):
        return utils.average(self.particles, self.weights)

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
        measurement_cov=measurement_cov)

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
