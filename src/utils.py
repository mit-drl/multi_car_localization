import math
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_id_to_index(num_cars, car_ids, ego_car):
    id_to_index = {}
    id_to_index[ego_car] = 0
    index = 1
    for car_id in car_ids:
        if car_id != ego_car:
            id_to_index[car_id] = index
            index += 1
    return id_to_index

def get_index_to_id(num_cars, car_ids, ego_car):
    index_to_id = {}
    index_to_id[0] = ego_car
    index = 1
    for car_id in car_ids:
        if car_id != ego_car:
            index_to_id[index] = car_id
            index += 1
    return index_to_id

def rk4(y0, u, dt, f):
    if dt == 0.0:
        return y0

    steps = 1.0
    h = dt/steps

    yn = y0

    for n in range(int(steps)):
        k1 = f(yn, u)
        k2 = f(yn + h*k1/2.0, u)
        k3 = f(yn + h*k2/2.0, u)
        k4 = f(yn + h*k3, u)

        yn = yn + (h/6.0)*(k1+2.0*k2+2.0*k3+k4)

    return yn

def euler(y0, u, dt, f):
    # doesn't seem to work
    dy = f(y0, u)
    yn = y0 + dt*dy
    return yn

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
        covs_xy = np.matmul(rot, np.matmul(covs[...,:2,:2], np.linalg.inv(rot)))
        covs_shape = covs_xy.shape
        covs_shape[-1] += 1
        covs_shape[-2] += 1
        new_covs = np.zeros(covs_shape)
        new_covs[...,:2,:2] = covs_xy
        new_covs[...,2,2] = covs[...,2,2]
    if return_covs:
        return new_poses, new_covs
    return new_poses

def transform(poses, origins):
    '''
    Transform poses from the coordinate frame defined by origins to the
    standard coordinate frame.
    '''
    xys, thetas = origins[...,:2], origins[...,2:]
    poses = rotate(poses, thetas)
    poses = poses + np.concatenate((xys, np.zeros_like(thetas)), axis=-1)
    return poses

def itransform(poses, origins):
    '''
    Do the inverse coordinate transform.
    '''
    xys, thetas = origins[...,:2], origins[...,2:]
    poses = poses - np.concatenate((xys, np.zeros_like(thetas)), axis=-1)
    poses = rotate(poses, -thetas)
    return poses

def average(poses, weights):
    mean = np.average(poses, axis=0, weights=weights)
    thetas = poses[:, 2]
    a_x = np.sum(np.cos(thetas) * weights)
    a_y = np.sum(np.sin(thetas) * weights)
    mean[2] = math.atan2(a_y, a_x)
    var = np.zeros((3, 3))
    var[:2, :2] = np.cov(poses[:,:2], ddof=0, aweights=weights, rowvar=False)
    # circular variance
    var[2, 2] = max(0, -math.log(float(a_x**2 + a_y**2) / np.sum(weights)))
    return mean, var

def directional_variance(cov, directions):
    xys = directions[...,:2]
    xys = xys / np.linalg.norm(xys, axis=-1, keepdims=True)
    return np.matmul(xys[...,None,:], np.matmul(cov[:2,:2], xys[...,:,None]))[...,0,0]

def make_pose(p):
    pose = Pose()
    quat = quaternion_from_euler(0, 0, p[2])
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def theta_from_quaternion(q):
    quaternion = (q.x, q.y, q.z, q.w)
    return euler_from_quaternion(quaternion)[2]

def quaternion_from_theta(theta):
    return quaternion_from_euler(0, 0, theta)

def pose_to_tf(pose, child_frame_id):
    tf = TransformStamped()
    tf.header = pose.header
    tf.child_frame_id = child_frame_id
    tf.transform.translation = pose.pose.position
    tf.transform.rotation = pose.pose.orientation
    return tf

def tf_to_pose(tf):
    pose = PoseStamped()
    pose.header = tf.header
    pose.pose.position = tf.transform.translation
    pose.pose.orientation = tf.transform.rotation
    return pose

def msg_to_tuple(msg):
    return tuple(getattr(msg, field) for field in msg.__slots__)

if __name__ == '__main__':
    poses = np.array([
        [2, 3, 0],
        [5, -5, np.pi/2],
    ])
    origin = np.array([4, -3, np.pi/2])
    print poses
    print transform(poses, origin)
    print itransform(transform(poses, origin), origin)
