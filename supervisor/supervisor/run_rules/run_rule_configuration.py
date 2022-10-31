from ..state.state_abc import StateABC
import math
from .rules import DistanceRule, LapRule, PenaltyRule, ReverseRule, TimeRule, UnmovingRule


class RunRuleConfiguration(StateABC):
    def __init__(self, parent):
        super().__init__(parent)

        self._configuration = {
            "distance": {
                "enabled": False,
                "distance_proportion_limit": 2
            },
            "reverse": {
                "enabled": False,
                "angle_limit": math.pi / 3 * 2
            },
            "lap": {
                "enabled": False,
                "lap_count_limit": 3
            },
            "time": {
                "enabled": False,
                "time_limit": 300
            },
            "penalty": {
                "enabled": False
            },
            "vehicle_unmoving": {
                "enabled": False,
                "time_limit": 10
            }
        }

        self._rule_lookup = {
            "distance": DistanceRule,
            "reverse": ReverseRule,
            "lap": LapRule,
            "time": TimeRule,
            "penalty": PenaltyRule,
            "vehicle_unmoving": UnmovingRule
        }

    @property
    def rules(self):
        return [self._rule_lookup[key](**conf) for key, conf in self._configuration.items() if key in self._rule_lookup]

    def set_configuration(self, conf):
        if conf:
            self._configuration = {
                key: {
                    c_key: conf.get(key, {}).get(c_key, c_value)
                    for c_key, c_value in value.items()
                }
                for key, value in self._configuration.items()
            }
        else:
            self._configuration = {
                key: {**c, "enabled": False}
                for key, c in self._configuration.items()}
        self.state_changed()
        return conf

    def to_dict(self):
        return self._configuration
