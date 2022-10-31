from .state_abc import StateABC
from fs_utils.conversions import map_to_dict, car_to_dict
from ..setup import Setup


class CurrentRun(StateABC):
    """
    Class to hold all the information about the current run
    """

    def __init__(self, parent, ros):
        super().__init__(parent)
        self.map = {}
        self.eval_map = {}
        self.gazebo_status = 0
        self.ready = False
        self.car = {}
        self.running = False
        self.was_reset = False
        self.driver = ""
        self.run_id = ""
        self.ros = ros
        self.ros.set_listener_lambda(
            Setup.Topics.SimulatorManager.gazebo_status,
            lambda _, msg: self._update_status(msg)
        )
        self.ros.set_listener_lambda(
            Setup.Topics.Eval.map,
            lambda _, msg: self._update_eval_map(msg)
        )

    def _update_status(self, msg):
        self.gazebo_status = msg.status
        if self.gazebo_status > 0:
            self.map = map_to_dict(msg.map)
            self.car = car_to_dict(msg.car) if msg.car.name else {}
            self.ready = bool(msg.car.name)
            self.running = msg.running
            self.was_reset = msg.was_reset
            self.driver = msg.driver
            self.run_id = msg.run_id
        else:
            self.map = {}
            self.eval_map = {}
            self.car = {}
            self.ready = False
            self.running = False
            self.was_reset = False
            self.run_id = ""

        self.state_changed()

    def _update_eval_map(self, msg):
        self.eval_map = {
            "centerline": map_to_dict(msg)["centerline"],
            "racingline": map_to_dict(msg)["yellow_cones"]
        }
        self.state_changed()

    def to_dict(self):
        return {
            "map": self.map,
            "eval_map": self.eval_map,
            "gazebo_status": self.gazebo_status,
            "car": self.car,
            "ready": self.ready,
            "running": self.running,
            "was_reset": self.was_reset,
            "driver": self.driver,
            "run_id": self.run_id
        }
