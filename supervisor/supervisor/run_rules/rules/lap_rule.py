from .rule_abc import RuleABC
from ...setup import Setup


class LapRule(RuleABC):
    def __init__(self, lap_count_limit=3, enabled=False):
        super().__init__("Lap limit exceeded", enabled)
        self._lap_count_limit = lap_count_limit

    def _is_triggered(self, topic, msg):
        if topic == Setup.Topics.Eval.lap:
            return msg.lap_count + 1 >= self._lap_count_limit
        return False
