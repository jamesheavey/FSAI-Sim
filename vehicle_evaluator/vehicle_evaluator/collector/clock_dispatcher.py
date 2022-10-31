class ClockDispatcher:
    def __init__(self, clock_callbacks=[]):
        self._clock_callbacks = clock_callbacks

    def handle_clock(self, msg):
        time = msg.clock.sec + msg.clock.nanosec / 1e9
        for callback in self._clock_callbacks:
            callback(time)
