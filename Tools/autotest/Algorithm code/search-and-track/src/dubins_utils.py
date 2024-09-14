import json
import os
import pickle
from datetime import datetime as dt

import dubins
import numpy as np


class DubinsBase:
    def __init__(self, Q: np.ndarray, turning_radius: float, **kwargs):
        """
        :param Q: (N, (2, 3)) a set of N configurations (in order) of the robot defined by (x, y, (z) \theta).
        :param turning_radius: turning radius of the dubins vehicle.
        :param **kwargs: any other keyword arguments.
        """
        self.Q = Q if isinstance(Q, np.ndarray) else np.array(Q)
        self.turning_radius = turning_radius
        self.__dict__.update(kwargs)

        # 3D treats the z dimension independently.
        if self.Q[0].size == 3:
            Q_pos = self.Q
        elif self.Q[0].size == 4:
            Q_pos = self.Q[:, [0, 1, 3]]

        # Paths cannot be saved as they are ctypes which can't be pickled.
        self.paths = []
        for q0, q1 in zip(Q_pos[:-1], Q_pos[1:]):
            path = dubins.shortest_path(q0, q1, turning_radius)
            self.paths.append(path)

    def curve_iterator(self):
        """Lazy generator to sample Dubins paths between keypoints and return
        only those segments.
        """
        for path in self.paths:
            yield np.array(path.sample_many(self.step_size)[0])


class DubinsMultiPath(DubinsBase):
    """Wrapper around the Dubins library:  https://pypi.org/project/dubins/
    to handle multiple dubins paths, similar to dubins.
    """

    def __init__(self, Q: np.ndarray, turning_radius: float, **kwargs):
        super(DubinsMultiPath, self).__init__(Q, turning_radius, **kwargs)

    def sample(self, step_size: float = None):
        """Given a dubins path, sample points at a fixed distance.

        TODO: this doesn't sample at a fixed distance, only approximately.

        :param step_size: size of each step to sample.
        :returns: (N, 2), xy array of points.
        """
        if step_size:
            self.step_size = step_size

        paths = [path.sample_many(self.step_size)[0] for path in self.paths]

        if len(paths) == 1:
            xy_points = np.array(paths)
        else:
            paths = [path for path in paths if path != []]
            xy_points = np.concatenate(paths)

        # Remove any points in the path that are less than a step size threshold
        mask = np.linalg.norm(xy_points[1:] - xy_points[:-1], axis=-1) > self.step_size / 1.1
        mask = np.concatenate(([True], mask), axis=0)
        xy_points = xy_points[mask.flatten()]

        return xy_points

    @property
    def path_length(self):
        """Exact path length of the Dubins segments."""
        return np.sum([path.path_length() for path in self.paths])
