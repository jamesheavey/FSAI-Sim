from ..setup import Setup


class ClockLatch:
    """
    Latch set if a clock signal arrives and reset manually
    Multiple individual latches are supported by providing the key argument
    """
    def __init__(self, ros):
        self.ros = ros
        self._latches = {}
        self.ros.set_listener_lambda(
            Setup.Topics.clock,
            self._clock_callback
        )

    def _clock_callback(self, _, __):
        for key in self._latches:
            self._latches[key] = True

    def reset(self, key="default"):
        self._latches[key] = False

    def is_set(self, key="default"):
        return self._latches.get(key, False)
