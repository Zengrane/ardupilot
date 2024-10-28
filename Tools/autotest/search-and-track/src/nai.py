import os
from copy import deepcopy
from typing import List, Union

import numpy as np
import toml
from shapely.geometry import MultiPolygon, Polygon

from utils import plot_polygon

from agent import CircularSensor, TriangleSensor


class NAI:
    """
    Named area of interest (NAI).
    Defines a region to search over.
    """

    def __init__(self, x_limits: Union[List, np.ndarray], y_limits: Union[List, np.ndarray]):
        """
        :param x_limits: (x_min, x_max)
        :param y_limits: (y_min, y_max)
        """
        self.x_limits = np.array(x_limits, dtype=float)
        self.y_limits = np.array(y_limits, dtype=float)

        boundary = np.array(
            [
                [x_limits[0], y_limits[0]],
                [x_limits[0], y_limits[1]],
                [x_limits[1], y_limits[1]],
                [x_limits[1], y_limits[0]],
                [x_limits[0], y_limits[0]],
            ]
        )
        self.boundary = Polygon(boundary)
        self.search_region = Polygon(boundary)

        # Plotting
        self.plots = {}

    @classmethod
    def from_toml(cls, fname: str = "etc/default.toml"):
        """Build the NAI from a toml config.

        :param fname: relative path to the toml config.
        """
        _cfg = toml.load(os.path.abspath(fname))
        return cls(_cfg["nai"]["boundary"]["x"], _cfg["nai"]["boundary"]["y"])

    def __copy__(self):
        nai = NAI(self.x_limits, self.y_limits)
        nai.search_region = deepcopy(self.search_region)
        return nai

    def apply_sensor(
        self,
        sensor: Union[CircularSensor, TriangleSensor],
        sensor_states: np.ndarray,
        tolerance: float = 0.1,
    ):
        """Apply the sensor over the search region.

        :param sensor: class representing the sensor
        :param sensor_states: (3,) (x, y, theta) of the sensor/robot.
        """
        for state in np.atleast_2d(sensor_states):
            poly = sensor.get_sensor(state)
            self.search_region = self.search_region.difference(poly)
            self.search_region.buffer(0)
        self.search_region = self.search_region.simplify(tolerance)

    def propagate(self, speed: float = 0.01):
        """Assumes the entities in the environment will move with some speed,
        therefore you need to slowly inflate the edges by that amount assuming
        holonomic dynamics.
        """
        self.search_region = self.search_region.buffer(speed)
        self.search_region = self.search_region.intersection(self.boundary)

    def draw(self, ax):
        self._draw_boundary(ax)
        self._draw_search_region(ax)

    def _draw_boundary(self, ax):
        """Draw the boundary of the NAI."""
        self.plots["nai_boundary"] = ax.plot(*np.array(self.boundary.exterior.xy), c="k", lw=1 / 2)

    def _draw_search_region(self, ax):
        """Draw the region that has NOT been searched over."""
        if "search_region" in self.plots:
            if isinstance(self.plots["search_region"], list):
                for poly_plot in self.plots["search_region"]:
                    poly_plot.remove()
            else:
                self.plots["search_region"].remove()

        if isinstance(self.search_region, MultiPolygon):
            poly_plots = []
            for poly in self.search_region.geoms:
                poly_plots.append(plot_polygon(ax, poly, fc="k", alpha=0.3))
            self.plots["search_region"] = poly_plots
        else:
            self.plots["search_region"] = plot_polygon(ax, self.search_region, fc="k", alpha=0.3)
