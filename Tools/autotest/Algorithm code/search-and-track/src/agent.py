import os
from copy import deepcopy
from typing import List, Union

import matplotlib as mpl
import numpy as np
import toml
from shapely.geometry import Point, Polygon

from dubins_utils import DubinsMultiPath
from utils import plot_polygon


class Agent:
    """Base class agent with bicycle kinematics."""

    def __init__(self, x0: np.ndarray, speed: float, max_turn_rate: float, dt: float):
        """
        :param x0: (x, y, theta) initial state.
        :param speed: speed of the vehicle.
        :param max_turn_rate: maximum turning rate (rad/s), larger == tighter circle.
        :param dt: update interval.
        """
        self.speed = speed
        self.max_turn_rate = max_turn_rate
        self.dt = dt

        # House keeping
        self.gt = 0.0
        self.X = np.atleast_2d(x0)
        self.plots = {}

    def step(self, x_t: Union[List, np.ndarray]):
        """Steps the agent forwards in the environment, given a new state x_t.

        :param x_t: new state.
        """
        self.gt += self.dt
        self.X = np.concatenate((self.X, np.atleast_2d(x_t)), axis=0)

    def step_by_u(self, u: np.ndarray):
        """Steps the agents forwards in the environment, given an array of control inputs u.

        :param u: (N,) array of controls.
        """
        u = np.clip(u, -self.max_turn_rate, self.max_turn_rate)
        X_ = np.atleast_2d(Agent.run_bicycle_kinematics(np.atleast_2d(u), self.speed, self.X[-1], self.dt))
        self.gt += len(u) * self.dt
        self.X = np.concatenate((self.X, X_), axis=0)

    @staticmethod
    def run_bicycle_kinematics(u: Union[List, np.ndarray], v: float, x0: np.ndarray, dt: float):
        """Bicycle kinematic model update.

        :param u: (..., 1) array of controls.
        :param v: speed of the bicycle.
        :param x0: initial configuration (x, y, theta)
        :returns: new configurations (x, y, theta).
        """
        theta = x0[..., -1].reshape(-1, 1, 1) + dt * np.cumsum(u, axis=-2)
        v = v * np.concatenate([np.cos(theta), np.sin(theta)], axis=-1)
        p = x0[..., :2].reshape(-1, 1, 2) + dt * np.cumsum(v, axis=-2)
        return np.squeeze(np.concatenate([p, theta], axis=-1))

    # ------------------
    # ---- Drawing  ----
    # ------------------
    def draw(self, ax, c: str = "k"):
        self._draw_marker(ax, c)
        self._draw_historical(ax, c)

    def draw_future_path(self, ax, plans: Union[List[np.ndarray], np.ndarray], c: str):
        """Draw the future path of the agent."""
        if "future" in self.plots:
            for p in self.plots["future"]:
                p.remove()

        plots = []
        for path in plans:
            plots.append(ax.plot(*path.T, c=c, alpha=0.2, zorder=100)[0])
        self.plots["future"] = plots

    def _draw_historical(self, ax, c: str = "k"):
        """Draw the historical position of the entity."""
        if "historical" in self.plots:
            self.plots["historical"].remove()

        self.plots["historical"] = ax.plot(*np.array(self.X)[..., :2].T, lw=1 / 2, c=c, alpha=0.05)[0]

    def _draw_marker(self, ax, c="k"):
        """Draw the current position of the Agent."""
        if "curr" in self.plots:
            self.plots["curr"].remove()

        marker, scale = Agent.triangle_marker(self.X[-1, -1])
        self.plots["curr"] = ax.scatter(*self.X[-1, :2], marker=marker, s=(30 * scale) ** 2, linewidths=0.0, c=c)

    @staticmethod
    def triangle_marker(rot: float):
        """
        :param rot: rotation from positive x-axis (rad).
        :returns: marker and scale factor
        """
        arrow = np.array([[-1 / 3, 1 / 3], [-1 / 3, -1 / 3], [2 / 3, 0]])
        rotation = np.array([[np.cos(rot), np.sin(rot)], [-np.sin(rot), np.cos(rot)]])
        arrow = np.matmul(arrow, rotation)
        marker = mpl.path.Path(arrow)
        scale = np.amax(
            np.abs([np.amin(arrow[:, 0]), np.amax(arrow[:, 0]), np.amin(arrow[:, 1]), np.amax(arrow[:, 1])])
        )

        return marker, scale


class Target(Agent):
    """Target we wish to find in the environment."""

    def __init__(self, x0, speed, max_turn_rate, dt):
        super(Target, self).__init__(x0, speed, max_turn_rate, dt)

        # For heuristics to make the target move
        self.spotted = False

    @classmethod
    def from_toml(cls, fname: str = "etc/default.toml"):
        """Build the Targets from a toml config.

        :param fname: relative path to the toml config.
        """
        _cfg = toml.load(os.path.abspath(fname))

        _targets = []
        for state in _cfg["target"]["kinematics"]["states"]:
            target = cls(
                state,
                _cfg["target"]["kinematics"]["speed"],
                _cfg["target"]["kinematics"]["max_turn_rate"],
                _cfg["sim"]["time"]["dt"],
            )
            _targets.append(target)

        return _targets

    def step(self):
        """Heuristic motion for the target. If spotted, then moves in a random manner."""
        if self.spotted:
            u = np.random.normal(0, 0.2, (1,))
            self.step_by_u(u)

    def take_measurement(self, sensor, sensor_state: np.ndarray, noise: float = 1.0):
        """Take a measurement of the target location.

        :param sensor: sensor object.
        :param sensor_state: (3, ) state of the sensor.
        :param noise: noise level.
        """
        if sensor.get_sensor(sensor_state).contains(Point(self.X[-1, :2])):
            return np.random.uniform(0, noise, (2,)) + self.X[-1, :2]
        return None

    def draw(self, ax):
        self._draw_marker(ax)
        self._draw_historical(ax, c="r")

    def _draw_marker(self, ax):
        """Draw position of the target.

        :param ax: axes to draw it on.
        """
        if "marker" in self.plots:
            self.plots["marker"].remove()
        self.plots["marker"] = ax.scatter(*self.X[-1, :2], s=20, c="r", marker="X")


class CEMAgent(Agent):
    """Agent that plans using CEM."""

    def __init__(self, x0, cfg, sensor, speed, max_turn_rate, dt, c: str = "b"):
        super(CEMAgent, self).__init__(x0, speed, max_turn_rate, dt)

        self.cfg = cfg
        self.c = c
        self.sensor = sensor

        # Planning
        self.current_plan = None
        self.reset_control()

    @classmethod
    def from_toml(cls, fname: str = "etc/default.toml"):
        """Build the searchers from a toml config.

        :param fname: relative path to the toml config.
        """
        _cfg = toml.load(os.path.abspath(fname))

        _searchers = []
        for state in _cfg["searcher"]["kinematics"]["states"]:
            sensor = None
            if _cfg["searcher"]["sensor"]["type"] == "triangle":
                sensor = TriangleSensor(_cfg["sensor"]["triangle"]["fov"], _cfg["sensor"]["triangle"]["range"])
            elif _cfg["searcher"]["sensor"]["type"] == "circle":
                sensor = CircularSensor(_cfg["sensor"]["circle"]["range"])

            searcher = cls(
                state,
                _cfg,
                sensor,
                _cfg["searcher"]["kinematics"]["speed"],
                _cfg["searcher"]["kinematics"]["max_turn_rate"],
                _cfg["sim"]["time"]["dt"],
            )
            _searchers.append(searcher)

        return _searchers

    def reset_control(self):
        """Resets the control parameters."""
        x_center = np.mean(self.cfg["nai"]["boundary"]["x"])
        y_center = np.mean(self.cfg["nai"]["boundary"]["x"])

        self.control_mean = np.repeat(
            np.array([x_center, y_center, 0], dtype=float)[np.newaxis], self.cfg["searcher"]["cem"]["n_params"], axis=0
        )
        self.control_var = np.repeat(
            np.array([x_center * 3, y_center * 3, 1], dtype=float)[np.newaxis],
            self.cfg["searcher"]["cem"]["n_params"],
            axis=0,
        )

    def step(self):
        """Steps the agent forwards in the environment based on the
        current plan.
        """
        self.gt += self.dt

        # Pop a planned state off the stack
        x_t = self.current_plan[0]
        self.current_plan = self.current_plan[1:]

        if len(self.current_plan) <= 1:
            self.current_plan = None
            self.reset_control()
            return True

        self.X = np.concatenate((self.X, np.atleast_2d(x_t)), axis=0)

        return False

    # -----------------
    # ---- Control ----
    # -----------------
    def sample_control(self, n_paths=100, use_mean=False):
        """Sample a set of n_paths.

        :param n_paths: number of paths.
        :param use_mean: if true, generates a single control sequence.
        :returns: list of paths and Dubins configurations.
        """
        if n_paths is None:
            n_paths = 1
        x_t = np.tile(self.X[-1], reps=(n_paths, 1))

        if use_mean:
            U = self.control_mean[np.newaxis, ...]
        else:
            U = np.random.normal(
                self.control_mean,
                np.sqrt(self.control_var),
                size=(n_paths, *self.control_mean.shape),
            )

        # Convert to Dubins curves
        max_idx = int(self.cfg["searcher"]["kinematics"]["max_path_length"] / (self.dt * self.speed))
        paths = [
            DubinsMultiPath(np.concatenate((np.atleast_2d(x_t), u), axis=0), self.max_turn_rate).sample(self.dt)[
                :max_idx
            ]
            for u in U
        ]

        return paths, U

    @staticmethod
    def deccem(
        agent,
        other_agents,
        reward_fn,
        use_mean=True,
    ):
        """Cross-entropy method for optimisation. MAXIMISES the reward function.

        :param agents: agent of interest to optimise over.
        :param other_agents: other agents distributions - do not optimise over them.
        :param reward_fn: reward function takes a set of paths and computes the cost
        :param use_mean: if True use the mean of the control paramters, rather than sample from the distribution.
        """
        n_iters = 0

        agent.control_var += 0.01

        while np.max(agent.control_var) > agent.cfg["searcher"]["cem"]["epsilon"]:
            n_iters += 1
            print(n_iters)
            if n_iters > agent.cfg["searcher"]["cem"]["communication_period"]:
                break

            # Generate a random sample from u ~ N(u_{t-1}, var_{t-1}).
            # U shape: (N_agents, N_samples, horizon, 1)
            # Plan shape: (N_agents, N_samples, horizon, 3)
            paths = []
            for _agent in [agent] + other_agents:
                plan, u = _agent.sample_control(n_paths=agent.cfg["searcher"]["cem"]["n_samples"])
                paths.append(plan)

                # Consider only your own actions.
                if _agent == agent:
                    U = u

            # Select elite trajectories
            rewards = reward_fn(paths)
            elite_idx = np.argsort(rewards, axis=0)[::-1][: agent.cfg["searcher"]["cem"]["n_elite"]]

            # Update mean, var based on elite trajectories
            elite_u = U[elite_idx]

            # Dynamic smoothing from "The Cross-Entropy Method for Continuous Multi-extremal Optimization"
            alpha = agent.cfg["searcher"]["cem"]["alpha"]
            beta_v = alpha
            # beta_v = beta - beta*(1 - 1/n_iters)**q

            # Smooth update based on alpha
            agent.control_mean = alpha * np.mean(elite_u, axis=0) + (1 - alpha) * agent.control_mean
            agent.control_var = beta_v * np.var(elite_u, axis=0) + (1 - beta_v) * agent.control_var

        # Set a new plan based on the updated controls
        agent.current_plan = np.squeeze(agent.sample_control(n_paths=1, use_mean=use_mean)[0])

    def draw(self, ax):
        self._draw_marker(ax, c=self.c)
        self.sensor.draw(ax, state=self.X[-1], c=self.c)
        self._draw_historical(ax, c=self.c)


class TriangleSensor:
    def __init__(self, fov: float, sensing_range: float):
        """
        :param fov: field-of-view of the sensor in degrees.
        :param sensing_range: maximum forwards range of the sensor.
        """
        self.fov = np.deg2rad(fov)  # rads
        self.sensing_range = sensing_range  # m
        self.area = self.get_sensor([0, 0, 0]).area
        self.plots = {}

    def __copy__(self):
        return TriangleSensor(self.fov, self.sensing_range)

    def get_sensor(self, state: np.ndarray):
        """
        :param state: (x, y, theta) state of the robot the sensor is attached to.
        """
        boundary = np.array(
            [
                [0, 0],
                [-1 * np.tan(self.fov / 2) * self.sensing_range, self.sensing_range],
                [np.tan(self.fov / 2) * self.sensing_range, self.sensing_range],
            ]
        )

        rot = state[2] - np.pi / 2
        rotation = np.array([[np.cos(rot), np.sin(rot)], [-np.sin(rot), np.cos(rot)]])
        boundary = np.matmul(boundary, rotation)
        boundary += state[:2]

        return Polygon(boundary)

    def draw(self, ax, state: np.ndarray, c: str):
        if "sensor" in self.plots:
            self.plots["sensor"].remove()
        self.plots["sensor"] = plot_polygon(ax, self.get_sensor(state), ec=c, fc="none")


class CircularSensor:
    def __init__(self, sensing_range: float):
        """
        :param sensing_range: maximum range of the sensor.
        """
        self.sensing_range = sensing_range
        self.area = np.pi * (sensing_range**2)
        self.plots = {}

    def __copy__(self):
        return CircularSensor(self.sensing_range)

    def get_sensor(self, state: np.ndarray):
        """
        :param state: (x, y, theta) state of the robot the sensor is attached to.
        """
        return Point(state[..., :2]).buffer(self.sensing_range)

    def draw(self, ax, state, c):
        if "sensor" in self.plots:
            self.plots["sensor"].remove()
        self.plots["sensor"] = plot_polygon(ax, self.get_sensor(state), ec=c, fc="none")
