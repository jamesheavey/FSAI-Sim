from .dispatcher import Dispatcher


class ClockDispatcher(Dispatcher):
    """
    Dispatcher implementation specific to clock
    Create a generic dispatcher, with a specific message transformer
    """

    def __init__(self, ros, topic, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            ClockDispatcher.clock_message_transformer,
            rate_reducer
        )

    @staticmethod
    def clock_message_transformer(msg):
        return msg.clock.sec + msg.clock.nanosec / 1e9
