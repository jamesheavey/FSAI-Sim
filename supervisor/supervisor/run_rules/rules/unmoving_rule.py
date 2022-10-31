from .rule_abc import RuleABC
from ...setup import Setup


class UnmovingRule(RuleABC):
    def __init__(self, time_limit=10, enabled=False):
        super().__init__("Vehicle unmoving time limit exceeded", enabled)
        self._time_limit = time_limit

    def _is_triggered(self, topic, msg):
        return topic == Setup.Topics.Eval.vehicle_unmoving and msg.data > self._time_limit
