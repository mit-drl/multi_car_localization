
import numpy as np


class JointState(np.ndarray):

    def get_state(self, i):
        return self[i]
