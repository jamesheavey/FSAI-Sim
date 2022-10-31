from .rule_abc import RuleABC
from ...setup import Setup


class DistanceRule(RuleABC):
    def __init__(self, distance_proportion_limit=2, enabled=False):
        super().__init__("Distance limit exceeded", enabled)
        self._distance_proportion_limit = distance_proportion_limit

    def _is_triggered(self, topic, msg):
        if topic == Setup.Topics.Eval.racingline_position:
            return msg.distance > msg.track_width * self._distance_proportion_limit
        return False
