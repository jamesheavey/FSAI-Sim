from .accumulator import PoseAccumulator, StringAccumulator, TwistAccumulator, Accumulator
from .accumulator import PenaltyAccumulator, RmsCrossTrackAccumulator
from .dispatcher import DispatcherABC
from .rate_reducer import TimedRateReducer
from ..setup import Setup


class DispatchHandler:
    def __init__(self, name, action):
        self._name = name
        self._action = action

    def __call__(self, *args):
        self._action(self._name, *args)


class SupervisorAccumulators(DispatcherABC):
    """
    Class to hold all supervisor accumulators
    """

    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.vehicle_position = PoseAccumulator(
            ros,
            Setup.Topics.odometry,
            rate_reducer=TimedRateReducer(0.05)
        )
        self.fuzzed_position = PoseAccumulator(
            ros,
            Setup.Topics.Fuzzed.odometry,
            rate_reducer=TimedRateReducer(0.05)
        )
        self.demo = StringAccumulator(ros, Setup.Topics.demo)
        self.vehicle_velocity = TwistAccumulator(
            ros,
            Setup.Topics.Eval.velocity,
            rate_reducer=TimedRateReducer(0.5)
        )
        self.vehicle_acceleration = TwistAccumulator(
            ros,
            Setup.Topics.Eval.acceleration,
            rate_reducer=TimedRateReducer(0.5)
        )
        self.driver_stdout = StringAccumulator(ros, Setup.Topics.Driver.stdout)
        self.lap_accumulator = Accumulator()
        self.penalty_accumulator = PenaltyAccumulator(ros, Setup.Topics.penalties)
        self.rms_cross_track_accumulator = RmsCrossTrackAccumulator(ros, Setup.Topics.Eval.racingline_position)

        self._accumulators = {
            "vehicle_position": self.vehicle_position,
            "fuzzed_position": self.fuzzed_position,
            "demo": self.demo,
            "vehicle_velocity": self.vehicle_velocity,
            "vehicle_acceleration": self.vehicle_acceleration,
            "driver_stdout": self.driver_stdout,
            "laps": self.lap_accumulator,
            "penalty": self.penalty_accumulator,
            "rms_cross_track": self.rms_cross_track_accumulator
        }
        for key, accumulator in self._accumulators.items():
            accumulator.subscribe(
                DispatchHandler(key, self._dispatch)
            )

    def get_all_data(self):
        return {key: accumulator.get_data() for key, accumulator in self._accumulators.items()}

    def handle_lap(self, lap_count, lap_time):
        lap_accumulators = [
            "vehicle_position",
            "vehicle_velocity",
            "vehicle_acceleration",
            "fuzzed_position",
            "rms_cross_track"
        ]

        data = {
            "lap": lap_count,
            "lap_time": lap_time,
            **{key: self._accumulators[key].get_data() for key in lap_accumulators}
        }
        for key in lap_accumulators:
            self._accumulators[key].clear()
        self.lap_accumulator.add_item(data)
