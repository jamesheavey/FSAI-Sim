from .rule_abc import RuleABC
from ...setup import Setup
import math


class ReverseRule(RuleABC):
    def __init__(self, angle_limit=math.pi / 3 * 2, enabled=False):
        super().__init__("Car angle limit exceeded", enabled)
        self._angle_limit = angle_limit

    def _is_triggered(self, topic, msg):
        if topic == Setup.Topics.Eval.racingline_position:
            return abs(msg.car_angle_to_racingline_angle) % math.pi > self._angle_limit
        return False
