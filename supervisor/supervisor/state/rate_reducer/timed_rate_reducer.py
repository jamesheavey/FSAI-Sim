from .rate_reducer_abc import RateReducerABC
import time


class TimedRateReducer(RateReducerABC):
    def __init__(self, min_delay):
        self._min_delay = min_delay
        self._last_triggered = time.time()

    def should_proceed(self):
        now = time.time()
        time_passed = (now - self._last_triggered) > self._min_delay
        if time_passed:
            self._last_triggered = now
        return time_passed
