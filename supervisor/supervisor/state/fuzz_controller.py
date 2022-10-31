from .state_abc import StateABC
from ..setup import Setup
import json
from collections import deque


class FuzzController(StateABC):
    """
    Class to interface with fuzzing module
    Hold initial configuration and changes in fuzzing as actions (time and new configuration)
    Hold actions in a queue dispatch them with time updates
    WHen reset set fuzzing to initial
    """

    def __init__(self, parent, ros):
        super().__init__(parent)

        self._ros = ros

        # Get default configuration
        self._ros.service_request(
            Setup.Services.Fuzzing.get_configuration,
            lambda resp: self.set_configuration({"initial": json.loads(resp.config_json)})
        )

        self._ros.set_listener_lambda(
            Setup.Topics.clock,
            lambda _, msg: self._clock_update(msg)
        )

        self._ros.set_listener_lambda(
            Setup.Topics.SimulatorManager.gazebo_status,
            lambda _, msg: self._update_status(msg)
        )

        self._prev_gazebo_status = -1
        self._initial_configuration = {}
        self._config_actions = []
        self._config_action_queue = deque()
        self.time = 0

    def set_configuration(self, config):
        """
        Update stored fuzzing configuration
        """
        initial = config.get("initial", {})
        if not self._initial_configuration or initial:
            self._initial_configuration = initial
        else:
            self._initial_configuration = {
                key: {**config, "fuzzing_enabled": False}
                for key, config in self._initial_configuration.items()}

        actions = config.get("actions", [])
        self._config_actions = [action for action in actions if "config" in action and "time" in action]
        self.state_changed()

    def apply_configuration(self):
        """
        Update currently set configuration and action queue
        """
        if all(action["time"] > self.time for action in self._config_actions):
            self._set_config(self._initial_configuration)

        self._config_action_queue = deque(
            sorted(self._config_actions, key=lambda action: action["time"], reverse=True)
        )

    def _clock_update(self, msg):
        """
        On clock update:
        store time, empty the relevant items from the queue,
        apply the latest action from the queue (avoid multiple assignments)
        """
        self.time = msg.clock.sec + msg.clock.nanosec / 1e9

        action = None
        while len(self._config_action_queue) > 0 and self._config_action_queue[-1]["time"] < self.time:
            action = self._config_action_queue.pop()
        if action:
            self._ros.publish(Setup.Topics.events, f"Fuzzing changed: {action['time']}")
            self._set_config(action["config"])

    def _update_status(self, msg):
        """
        Handle gazebo status updates
        """
        if msg.status != self._prev_gazebo_status or msg.was_reset:
            self._reset()
        self._prev_gazebo_status = msg.status

    def _reset(self):
        """
        Handle reset
        """
        self.time = 0
        self._ros.service_request(
            Setup.Services.Fuzzing.reset,
            lambda res: self._ros.log.info(f"Fuzz reset {res}"),
        )
        self.apply_configuration()

    def _set_config(self, config):
        """
        Thin wrapper around calling the set_configuration service
        """
        return self._ros.service_request(
            Setup.Services.Fuzzing.set_configuration,
            lambda res: self._ros.log.info(f"Fuzz set {res}"),
            config
        )

    def to_dict(self):
        return {
            "initial": self._initial_configuration,
            "actions": self._config_actions
        }
