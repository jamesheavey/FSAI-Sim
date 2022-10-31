from .dispatcher import MetadataDispatcher, DispatcherABC, ClockDispatcher, VehiclePoseOnMapDispatcher, StringDispatcher
from ..setup import Setup
from .rate_reducer import CounterRateReducer


class SupervisorDispatchers(DispatcherABC):
    """
    Class to hold all the supervisor dispatchers
    """

    def __init__(self, ros):
        super().__init__()
        self.driver_metadata = MetadataDispatcher(
            ros,
            Setup.Topics.Driver.metadata,
            # rate_reducer=CounterRateReducer(5)
        )
        self.driver_metadata.subscribe(
            lambda *args, **kwargs: self._dispatch(
                f"driver_metadata_{kwargs.get('source', '')}", *args, **kwargs)
        )
        self.clock = ClockDispatcher(
            ros,
            Setup.Topics.clock,
            rate_reducer=CounterRateReducer(1)
        )
        self.clock.subscribe(
            lambda *args, **kwargs: self._dispatch(
                "clock", *args, **kwargs)
        )
        self.racingline_position = VehiclePoseOnMapDispatcher(
            ros,
            Setup.Topics.Eval.racingline_position
        )
        self.racingline_position.subscribe(
            lambda *args, **kwargs: self._dispatch(
                "racingline_position", *args, **kwargs)
        )
        self.events = StringDispatcher(
            ros,
            Setup.Topics.events
        )
        self.events.subscribe(
            lambda *args, **kwargs: self._dispatch(
                "events", *args, **kwargs)
        )
