from .rule_abc import RuleABC
from ...setup import Setup


class PenaltyRule(RuleABC):
    def __init__(self, enabled=False):
        super().__init__("Penalty issued", enabled)

    def _is_triggered(self, topic, msg):
        return topic == Setup.Topics.penalties
