from ..setup import Setup
from .dispatcher import DispatcherABC


class ClockAggregator(DispatcherABC):
    def __init__(self, ros, rate_reducer=None):
        super().__init__()
        self.ros = ros
        self._snapshot = []
        self.ros.set_listener_lambda(
            Setup.Topics.clock,
            self._time_handler
        )
        self._rate_reducer = rate_reducer

    def dispatcher_handler(self, name, data, *args, **kwargs):
        self._snapshot.append({"actionType": "dispatcher", "data": {"name": name, "data": data}})

    def accumulator_handler(self, name, action, data, *args, **kwargs):
        self._snapshot.append({"actionType": "accumulator", "data": {"name": name, "action": action, "data": data}})

    def _time_handler(self, _, msg):
        time = msg.clock.sec + msg.clock.nanosec / 1e9
        self._dispatch_snapshot(time)

    def _dispatch_snapshot(self, time):
        if self._rate_reducer and not self._rate_reducer.should_proceed():
            return

        self._dispatch({
            "time": time,
            "actions": self._snapshot
        })
        self._snapshot = []

    def force_dispatch(self, time=0):
        self._dispatch({
            "time": time,
            "actions": self._snapshot
        })
        self._snapshot = []
