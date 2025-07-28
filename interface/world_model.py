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
    def get_distribution(self) -> List[NDArray]:
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
        """

        Args:
            save_path:
            rollouts_per_instance:
        """

        super().__init__()
        self.obstacles = {}  # {"SAND": [2.4, 3.4]}
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
        self.samples = []

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

        pths = glob.glob(self.save_path.as_posix() + "/*.npy")
        pths.sort()
        self.samples = [np.load(d, allow_pickle=True) for d in pths]
        print("saved samples to World Model")

    def get_distribution(self) -> List[NDArray]:
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
        return self.samples

    def get_distribution_at_time(self, time: float) -> NDArray[float]:
        """
        Compute the empirical state distribution given the latest rollout.

        Distribution is [\mu, \sigma] for a subset of state elements [t, x, y]

        State = [
            x, y, z,
            rx, ry, rz, angle,
            num observed obstacles,
            robot time
            wall time
            ]
        Args:
            time:

        Returns:
            NDArray[state element, distribution]
        """

        def find_nearest(array, value):
            idx = (np.abs(array - value)).argmin()
            return idx

        idxs = [find_nearest(x[:, 8], time) for x in self.samples]
        t_pred = [x[idx, 8] for x, idx in zip(self.samples, idxs)]
        x_pred = [x[idx, 0] for x, idx in zip(self.samples, idxs)]
        y_pred = [x[idx, 1] for x, idx in zip(self.samples, idxs)]
        return np.array(
            [
                [np.mean(t_pred), np.std(t_pred)],
                [np.mean(x_pred), np.std(x_pred)],
                [np.mean(y_pred), np.std(y_pred)],
            ]
        )

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


if __name__ == "__main__":
    wm = WebotsWorldModel(Path(__file__).parent.parent / "rollouts")
    s = State(
        position=np.array([0, 0, 0.0]),
        orientation=np.array([0, 0, 0.0, 0]),
        time=0.0,
        goal=np.array([2, 5.0]),
        known_obstacles={},
        waypoints=np.array([[0, 1], [1, 3], [2, 5]]),
    )
    wm.generate_samples(s, s.waypoints)
    import matplotlib.pyplot as plt

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect("equal")
    for i in range(1, 25):
        pred = wm.get_distribution_at_time(i)
        pred_x = pred[0]
        pred_y = pred[1]
        plt.scatter(pred_x[0], pred_y[0])
        plt.pause(0.01)
    plt.show()
