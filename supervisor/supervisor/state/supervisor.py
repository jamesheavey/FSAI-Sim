from .dispatcher import DispatcherABC
from .state_abc import StateABC
from .setup_options import SetupOptions
from .current_run import CurrentRun
from .image_store import ImageStore
from .supervisor_dispatchers import SupervisorDispatchers as Dispatchers
from .supervisor_accumulators import SupervisorAccumulators as Accumulators
from ..setup import Setup
from .clock_aggregator import ClockAggregator
from .rate_reducer import TimedRateReducer
from .reports import Reports
from ..run_rules import RunRuleConfiguration
from .clock_latch import ClockLatch
from .fuzz_controller import FuzzController


class Supervisor(StateABC, DispatcherABC):
    """
    Central state container
    Holds states, large set of infreqnetly updated data
    Holds accumulators and dispatchers
    Holds image stores
    """

    def __init__(self, ros):
        StateABC.__init__(self, parent=None)
        DispatcherABC.__init__(self)
        self.ros = ros

        self.current_run = CurrentRun(self, ros)
        self.options = SetupOptions(self, ros)
        self.reports = Reports(self, ros)
        self.fuzz_controller = FuzzController(self, ros)

        self.image_stores = {
            "camera1_raw": ImageStore(ros, Setup.Topics.Camera1.raw),
            "camera1_labelled": ImageStore(ros, Setup.Topics.Camera1.labelled),
            "camera1_fuzzed": ImageStore(ros, Setup.Topics.Camera1.fuzzed),
            "camera2_raw": ImageStore(ros, Setup.Topics.Camera2.raw),
            "camera2_labelled": ImageStore(ros, Setup.Topics.Camera2.labelled),
            "camera2_fuzzed": ImageStore(ros, Setup.Topics.Camera2.fuzzed),
            "camera0_raw": ImageStore(ros, Setup.Topics.Camera0.raw),
            "camera0_labelled": ImageStore(ros, Setup.Topics.Camera0.labelled),
            "camera0_fuzzed": ImageStore(ros, Setup.Topics.Camera0.fuzzed),
            "camera_3pv": ImageStore(ros, Setup.Topics.Camera3PV.raw),
        }

        self.accumulators = Accumulators(ros)
        self.dispatchers = Dispatchers(ros)

        self.aggregator = ClockAggregator(ros, rate_reducer=TimedRateReducer(0.2))
        self.accumulators.subscribe(self.aggregator.accumulator_handler)
        self.dispatchers.subscribe(self.aggregator.dispatcher_handler)

        self.ros.set_listener_lambda(
            Setup.Topics.Eval.lap,
            self._handle_lap
        )

        self.run_rule_configuration = RunRuleConfiguration(self)
        self.clock_latch = ClockLatch(ros)

    def state_changed(self):
        self.ros.log.info("State changed")
        self._dispatch(self)

    def _handle_lap(self, _, lap):
        self.accumulators.handle_lap(lap.lap_count, lap.lap_time)
        self.aggregator.force_dispatch()

    def to_dict(self):
        return {
            "current_run": self.current_run.to_dict(),
            "options": self.options.to_dict(),
            "reports": self.reports.to_dict(),
            "run_rules": self.run_rule_configuration.to_dict(),
            "fuzz_controller": self.fuzz_controller.to_dict()
        }

    def reset(self):
        self.accumulators.vehicle_position.clear()
        self.accumulators.fuzzed_position.clear()
        self.accumulators.vehicle_velocity.clear()
        self.accumulators.vehicle_acceleration.clear()
        self.dispatchers.driver_metadata.clear()
        self.accumulators.driver_stdout.clear()
        self.accumulators.lap_accumulator.clear()
        self.accumulators.penalty_accumulator.clear()
        self.accumulators.rms_cross_track_accumulator.clear()
        self.aggregator.force_dispatch()
