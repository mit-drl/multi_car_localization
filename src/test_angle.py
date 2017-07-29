import numpy as np
import math

angles = np.array([90.0, -10.0])
weights = np.array([0.66, 0.33])

def norm_weights(weights):
	weights = weights/weights.sum()
	return weights

weights = norm_weights(weights)

def mean_angle(angles, weights):
    x = y = 0.
    for angle, weight in zip(angles.tolist(), weights.tolist()):
        x += math.cos(math.radians(angle)) * weight
        y += math.sin(math.radians(angle)) * weight

    mean = math.degrees(math.atan2(y, x))
    return mean

mean_angle = mean_angle(angles, weights)

print mean_angle