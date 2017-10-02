import math
import numpy as np

def rotate(poses, phis, covs=None, return_covs=False):
    # construct rotation matrices
    rot = np.array([[np.cos(phis), -np.sin(phis)],
                    [np.sin(phis), np.cos(phis)]])[...,0]
    rot = np.transpose(rot, axes=range(2,rot.ndim)+[0,1])
    # apply rotation matrices to measurements
    xys = np.matmul(rot, poses[..., :2, None])[..., 0]
    thetas = poses[..., 2:] + phis
    new_poses = np.concatenate((xys, thetas), axis=-1)
    # if we have covariance matrices, rotate those too
    new_covs = None
    if covs is not None:
        covs_xy = np.matmul(rot, np.matmul(covs[...,:2:2], np.linalg.inv(rot)))
        covs_shape = covs_xy.shape
        covs_shape[-1] += 1
        covs_shape[-2] += 1
        new_covs = np.zeros(covs_shape)
        new_covs[...,:2,:2] = covs_xy
        new_covs[...,2,2] = covs[...,2,2]
    if return_covs:
        return new_poses, new_covs
    return new_poses

def average(poses, weights):
    mean = np.average(poses, axis=0, weights=weights)
    thetas = poses[:, 2]
    a_x = np.sum(np.cos(thetas) * weights)
    a_y = np.sum(np.sin(thetas) * weights)
    mean[2] = math.atan2(a_y, a_x)
    var = np.zeros((3, 3))
    var[:2, :2] = np.cov(poses[:,:2].T, aweights=weights)
    # circular variance
    var[2, 2] = -math.log(float(a_x**2 + a_y**2) / np.sum(weights))
    return mean, var

def directional_variance(cov, directions):
    xys = directions[...,:2]
    xys = xys / np.linalg.norm(xys, axis=-1, keepdims=True)
    return np.matmul(xys[:,None,:], np.matmul(cov[:2,:2], xys[:,:,None]))[...,0,0]
