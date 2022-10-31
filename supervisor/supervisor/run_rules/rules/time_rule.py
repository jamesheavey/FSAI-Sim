from .rule_abc import RuleABC
from ...setup import Setup


class TimeRule(RuleABC):
    def __init__(self, time_limit=300, enabled=False):
        super().__init__("Time limit exceeded", enabled)
        self._time_limit = time_limit

    def _is_triggered(self, topic, msg):
        if topic == Setup.Topics.clock:
            return msg.clock.sec + msg.clock.nanosec / 1e9 > self._time_limit
        return False
