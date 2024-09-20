import numpy as np
import ray
from copy import copy

class NAI:
    @staticmethod
    def from_toml(config_fname):
        # Load NAI configuration from a TOML file
        # This is a placeholder; replace with actual loading logic
        nai_instance = NAI()
        # Load necessary parameters
        nai_instance.boundary = np.array([0, 0, 100, 100])  # Example boundary
        nai_instance.search_region = np.array([0, 0, 50, 50])  # Example search region
        return nai_instance

class Target:
    @staticmethod
    def from_toml(config_fname):
        # Load targets from a TOML file
        # This is a placeholder; replace with actual loading logic
        return [Target() for _ in range(3)]  # Example: create 3 targets

    def take_measurement(self, sensor, position, noise=0.0):
        # Simulate taking a measurement of the target's state
        # This is a placeholder; replace with actual measurement logic
        return np.array([position[0] + noise, position[1] + noise])  # Example measurement

class CEMAgent:
    @staticmethod
    def from_toml(config_fname):
        # Load agents from a TOML file
        # This is a placeholder; replace with actual loading logic
        return [CEMAgent() for _ in range(3)]  # Example: create 3 agents

    def generate_plans(self):
        # Generate potential plans for the agent
        # This is a placeholder; replace with actual planning logic
        return np.random.rand(10, 3)  # Example: 10 plans with 3 dimensions

    def update_based_on_rewards(self, reward):
        # Update the agent's behavior based on the computed reward
        # This is a placeholder; implement reward application logic
        pass

def _target_coverage(plans, beliefs, sensors):
    cost = 0
    for belief in beliefs:
        for plan, sensor in zip(plans, sensors):
            belief.simulate_path(sensor, plan)
        cost += -1 * (belief.belief.area / np.sum([sensor.area for sensor in sensors]))
    return cost

def _nai_coverage(plans, nai, sensors):
    for plan, sensor in zip(plans, sensors):
        nai.apply_sensor(sensor, plan)
    return -1 * nai.search_region.area / nai.boundary.area

def _avoid_threat_zones(plans, beliefs):
    cost = 0
    for plan in plans:
        cost -= np.sum(np.linalg.norm(plan[..., :2] - np.array([15, 15], dtype=float), axis=-1) > 5)
    return cost

@ray.remote
def _reward(idx, plans, nai, beliefs, sensors):
    cost = 0

    # Target belief cost
    if beliefs:
        cost += _target_coverage(plans, beliefs, sensors)

    # Avoid threat zones
    if _avoid_threat_zones(plans, beliefs) > 0:
        cost = 0

    return {"id": idx, "reward": cost}

def reward_fn(plans, nai, beliefs, agents):
    ray_ids = []

    for p_idx in range(len(plans[0])):
        _beliefs = beliefs
        if len(_beliefs) > 0:
            _beliefs = [copy(belief) for belief in beliefs if belief]

        plan = []
        for a_idx in range(len(plans)):
            plan.append(plans[a_idx][p_idx])

        sensors = [copy(agent.sensor) for agent in agents]

        ray_ids.append(_reward.remote(p_idx, plan, copy(nai), _beliefs, sensors))

    results = ray.get(ray_ids)

    return [r["reward"] for r in sorted(results, key=lambda x: x["id"])]
