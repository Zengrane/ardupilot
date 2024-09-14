import itertools as it
import os
from copy import copy, deepcopy

import click
import matplotlib.pyplot as plt
import numpy as np
import ray
import toml
from matplotlib.animation import FFMpegWriter

from agent import CEMAgent, Target
from belief import Belief
from nai import NAI

from shapely.geometry import LineString


def _target_coverage(plans, beliefs, sensors):
    """Computes the coverage of the target beliefs as a percentage of the
    sensor area.
    """
    cost = 0
    for belief in beliefs:
        for plan, sensor in zip(plans, sensors):
            belief.simulate_path(sensor, plan)
        cost += -1 * (belief.belief.area / np.sum([sensor.area for sensor in sensors]))
    return cost


def _nai_coverage(plans, nai, sensors):
    """Computes the coverage of the NAI as a percentage of the total area."""
    for plan, sensor in zip(plans, sensors):
        nai.apply_sensor(sensor, plan)
    return -1 * nai.search_region.area / nai.boundary.area


def _avoid_threat_zones(plans, beliefs):
    """Negatively reward the threat zones"""
    cost = 0
    for plan in plans:
        cost -= np.sum(
            np.linalg.norm(plan[..., :2] - np.array([15, 15], dtype=float), axis=-1) > 5
        )

    return cost


@ray.remote
def _reward(idx, plans, nai, beliefs, sensors):
    cost = 0

    # # Coverage cost
    # cost += _nai_coverage(plans, nai, sensors)

    # Target belief cost
    if len(beliefs) > 0:
        cost += _target_coverage(plans, beliefs, sensors)

    # Avoid threat zones
    if _avoid_threat_zones(plans, beliefs) > 0:
        cost = 0
    # cost += _avoid_threat_zones(plans, beliefs)

    return {"id": idx, "reward": cost}


def reward_fn(plans, nai, beliefs, agents):
    """Reward function for centralised CEM opt.

    :param plans: list of lists of (n agents, n samples, horizon of each sample, 3)
    :param nai: -
    :param beliefs: -
    :param agents: list of agents
    """
    ray_ids = []

    for p_idx in range(len(plans[0])):
        # Copy target beliefs
        _beliefs = beliefs
        if len(_beliefs) > 0:
            _beliefs = [copy(belief) for belief in beliefs if belief]

        # Restack plans
        plan = []
        for a_idx in range(len(plans)):
            plan.append(plans[a_idx][p_idx])

        # Copy sensors
        sensors = [copy(agent.sensor) for agent in agents]

        # Compute the reward
        ray_ids.append(_reward.remote(p_idx, plan, copy(nai), _beliefs, sensors))

    results = ray.get(ray_ids)

    return [r["reward"] for r in sorted(results, key=lambda x: x["id"])]


@click.command()
@click.option(
    "--config_fname",
    type=str,
    help="Path to the *.toml configuration.",
    default="etc/default.toml",
)
@click.option(
    "--output",
    type=str,
    help="Filename without ext. of the output video.",
    default="result",
)
def main(config_fname: str, output: str):
    # Startup
    ray.init()

    # Config
    _cfg = toml.load(os.path.abspath(config_fname))

    # Setup
    nai = NAI.from_toml(config_fname)
    targets = Target.from_toml(config_fname)
    agents = CEMAgent.from_toml(config_fname)

    # Target beliefs are shared amongst searchers
    beliefs = [None for _ in targets]

    # Plotting
    fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    ax = np.atleast_1d(ax)
    ax[0].set_xlim(_cfg["nai"]["boundary"]["x"])
    ax[0].set_ylim(_cfg["nai"]["boundary"]["y"])
    ax[0].set_aspect("equal")
    plt.tight_layout()

    # Simulation loop
    writer = FFMpegWriter(fps=10)
    with writer.saving(fig, f"../results/{output}.mp4", 250):
        while agents[0].gt <= _cfg["sim"]["time"]["max_T"]:
            print("------------ Replanning... ------------")
            # Reset all the controls for simplicity.
            for agent in agents:
                agent.reset_control()

            # Dec-CEM planning
            for _ in range(_cfg["searcher"]["cem"]["communication_cycles"]):
                for a_idx, agent in enumerate(agents):
                    CEMAgent.deccem(
                        agent,
                        agents[:a_idx] + agents[a_idx + 1 :],
                        lambda paths: reward_fn(paths, nai, beliefs, agents),
                        use_mean=True,
                    )

            # Quick logic for replanning
            # - If target has entered and LEFT your FoV, replan
            # - If you run out of plan, replan
            plan_done = False
            target_in_fov = False
            while not plan_done:
                # Agent
                print(f"gt: {agents[0].gt}")
                for agent in agents:
                    plan_done |= agent.step()
                    agent.draw(ax[0])
                    if isinstance(agent.current_plan, np.ndarray):
                        agent.draw_future_path(
                            ax[0], agent.current_plan[np.newaxis, ..., :2], c=agent.c
                        )

                # NAI
                for agent in agents:
                    nai.apply_sensor(agent.sensor, agent.X[-1])
                nai.draw(ax[0])

                # Belief updates
                in_range = False
                for t_i, target in enumerate(targets):
                    for _, agent in enumerate(agents):
                        measurement = target.take_measurement(
                            agent.sensor, agent.X[-1], noise=0.0
                        )
                        if isinstance(measurement, np.ndarray):
                            in_range = True
                            if beliefs[t_i] is None:
                                beliefs[t_i] = Belief.from_measurement(
                                    measurement, target.speed
                                )
                                target.spotted = True
                            else:
                                beliefs[t_i].update_measurement(measurement)
                        else:
                            if beliefs[t_i]:
                                beliefs[t_i].apply_sensor(agent.sensor, agent.X[-1])

                # Target ground-truth and belief
                for target, belief in zip(targets, beliefs):
                    # Ground truth
                    target.step()
                    target.draw(ax[0])

                    # Belief plots
                    if belief:
                        belief.propagate()
                        belief.draw(ax[0])

                if in_range and not target_in_fov:
                    target_in_fov = True  # Replan before end of the path
                    target_in_fov = False  # Do not replan before the end of the path

                if target_in_fov and not in_range:
                    break

                writer.grab_frame()
                plt.pause(0.1)


if __name__ == "__main__":
    main()
