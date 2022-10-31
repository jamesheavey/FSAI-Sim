import numpy as np


class VehicleVec:
    """
    Vector 2, that can be converted to lat and lon given the frame
    """

    def __init__(self, vec, frame):
        self._vec = vec
        self._frame = frame

    @property
    def x(self):
        return self._vec[0]

    @property
    def y(self):
        return self._vec[1]

    @property
    def z(self):
        return self._vec[2]

    @property
    def value(self):
        return self._vec

    @property
    def longitudinal(self):
        if self._frame is None:
            return 0.0
        forward = np.matmul(self._frame[0:3, 0:3], np.array([1, 0, 0]))
        return np.dot(self._vec, forward)

    @property
    def lateral(self):
        if self._frame is None:
            return 0.0
        right = np.matmul(self._frame[0:3, 0:3], np.array([0, 1, 0]))
        return np.dot(self._vec, right)
