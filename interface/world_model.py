import glob
import subprocess
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml
from numpy.typing import NDArray


@dataclass
class State:
    position: NDArray[float]
    orientation: NDArray[float]
    time: float
    goal: NDArray[float]
    known_obstacles: Dict[str, NDArray[float]]
    waypoints: Optional[NDArray]


class WorldModel(ABC):
    @abstractmethod
    def generate_samples(self, starting_state: State, path: NDArray[float]) -> None:
        pass

    @abstractmethod
    def get_samples(self) -> List[NDArray]:
        pass

    @abstractmethod
    def get_distribution_at_time(self, time: float) -> NDArray[float]:
        """
        Computes a state distribution [mu, sigma] for each element of the state vector at
        the specified time.

        Args:
            time:   The time to sample at

        Returns:
            Distribution NDArray[state_element, mu, sigma]
        """


class WebotsWorldModel(WorldModel):
    """
    A World Model implementation using the Webots simulator as the model.
    """

    def __init__(self, save_path: Path, rollouts_per_instance: int = 10) -> None:
        super().__init__()
        self.obstacles = {"SAND": [2.4, 3.4]}
        self.save_path = save_path
        self.num_rollouts_per_instance = rollouts_per_instance
        self.world_path = (
            Path(__file__).parent.parent
            / "webots_world_model"
            / "worlds"
            / "rollout_simulation.wbt"
        )
        self.settings_path = (
            Path(__file__).parent.parent
            / "webots_world_model"
            / "controllers"
            / "rollout_controller"
            / "settings.yaml"
        )

    def generate_samples(self, current_sate: State, path: NDArray[float]) -> None:
        """
        Generate sample rollouts from this World Model.

        Args:
            current_sate:   The current state of the robot.
            path:           The waypoint path the robot seeks to achieve.

        Returns:
            None
        """
        self._do_rollouts(current_sate, 0, path)

    def get_samples(self) -> List[NDArray]:
        """
        Get all the samples from the latest World Model rollout.

        Samples are of the form list[array[timesteps, state]] where each
        element of the list is a rollout of size [timesteps, state]. So,
        for a rollout of size 5, expect the following sizes:
            - len(predicted_states) == 5, the number of rollouts.
            - len(predicted_states[0]) == N, where N is the number of
                timesteps in a single rollout.
            - len(predicted_states[0][0]) = 10, the size of the returned
                state vector.

        Returns:
            Samples are of the form list[array[timesteps, state]]
        """
        pths = glob.glob(self.save_path.as_posix() + "/*.npy")
        pths.sort()
        predicted_states = [np.load(d, allow_pickle=True) for d in pths]
        return predicted_states

    def _do_rollouts(
        self, current_sate: State, waypoint_counter: int, waypoints: NDArray[float]
    ) -> None:
        """
        Execute Monte-Carlo rollouts of the Webots World Model given some initial information.

        Args:
            current_sate:       The current state of the robot
            waypoint_counter:   The index into waypoints that the robot is currently on.
            waypoints:          The waypoint path the robot is attempting to achieve.

        Returns:
            None
        """
        configs = {
            "start_position": [float(x) for x in current_sate.position],
            "start_orientation": [float(x) for x in current_sate.orientation],
            "goal_position": [float(x) for x in current_sate.goal],
            "waypoint_index": waypoint_counter,
            "num_iterations": self.num_rollouts_per_instance,
            "publish": False,
            "prefix": "rollout",
            "known_obs": self.obstacles,
            "save_path": self.save_path.absolute().as_posix(),
        }
        if waypoints is not None:
            configs["wp_x"] = [float(x) for x in waypoints[:, 0]]
            configs["wp_y"] = [float(x) for x in waypoints[:, 1]]

        print(yaml.dump(configs))
        with open(self.settings_path.absolute().as_posix(), "w") as f:
            yaml.dump(configs, f, default_flow_style=None, sort_keys=False)

        command = [
            "webots",
            self.world_path.absolute().as_posix(),
            "--minimize",
            "--batch",
            "--mode=fast",
            "--stdout",
        ]
        p = subprocess.Popen(command)
        stdout, stderr = p.communicate()
