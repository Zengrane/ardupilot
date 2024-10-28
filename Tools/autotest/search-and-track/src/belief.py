from copy import deepcopy
from typing import List, Union

import numpy as np
from shapely.geometry import Polygon, MultiPolygon, Point

from utils import plot_polygon


class Belief:
    """Rather than storing a target as a GT target, you maintain a belief over your observations
    where those observations are filtered by a tracker of some description. From that
    tracker you generate the spread of where it could be when unobserved.
    """

    def __init__(self, belief: Union[Polygon, MultiPolygon], speed: float = 0.03):
        """
        :param belief: Polygon of the possible target locations.
        :param speed: speed of the target, assuming holonomic dynamics.
        """
        self.belief = belief
        self.speed = speed
        self.plots = {}

    def __copy__(self):
        """Return a copy of the object, without having to deepcopy the entire structure."""
        return Belief(deepcopy(self.belief), self.speed)

    @classmethod
    def from_measurement(cls, xy: np.ndarray, speed: float):
        """Belief stems from a measured location.

        :param xy: xy position.
        :param speed: maximum speed of the target under track.
        :returns: Belief over positions.
        """
        belief = Point(xy).buffer(speed)
        return cls(belief, speed)

    def update_measurement(self, xy: Union[List, np.ndarray]):
        """Updates the estimated state of the target.

        :param xy: xy position of the target.
        """
        self.belief = Point(xy).buffer(self.speed)

    def apply_sensor(self, sensor: Union[Polygon, MultiPolygon], sensor_states: np.ndarray):
        """Apply the sensor over the belief and truncate it based on that.

        :param sensor: class representing the sensor
        :param sensor_states: (3,) (x, y, theta) of the sensor/robot.
        """
        for state in np.atleast_2d(sensor_states):
            poly = sensor.get_sensor(state)
            self.belief = self.belief.difference(poly)

            # TODO: partial fix for the nested shell geometry, still raises.
            self.belief.buffer(0)

    def propagate(self):
        """Propagate where we believe the object of interest to be; in this case,
        it is bounded by the maximum speed of the object.
        """
        self.belief = self.belief.buffer(self.speed)

    def simulate_path(self, sensor: Union[Polygon, MultiPolygon], path: Union[List, np.ndarray]):
        """Simultaneously applies a sensor and updates the belief.

        NOTE: this function modifies the underlying state, so use a copy of the object.

        :param sensor: -
        :param path: (..., (2, 3)) array of states the sensor occupies.
        """
        for state in np.atleast_2d(path):
            self.apply_sensor(sensor, state)
            self.propagate()

    def draw(self, ax):
        """Draw the belief of the target."""
        if "belief" in self.plots:
            if isinstance(self.plots["belief"], list):
                for poly_plot in self.plots["belief"]:
                    poly_plot.remove()
            else:
                # TODO: remove ducks
                try:
                    self.plots["belief"].remove()
                except:
                    pass

        if self.belief:
            if isinstance(self.belief, MultiPolygon):
                self.plots["belief"] = [
                    plot_polygon(ax, poly, fc="r", alpha=0.3, zorder=100) for poly in self.belief.geoms
                ]
            else:
                self.plots["belief"] = plot_polygon(ax, self.belief, fc="r", alpha=0.3, zorder=100)
