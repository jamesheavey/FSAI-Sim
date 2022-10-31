from .dispatcher import Dispatcher


class StringDispatcher(Dispatcher):
    """
    Dispatcher implementation specific to String
    Create a generic dispatcher, with a specific message transformer
    """

    def __init__(self, ros, topic, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            lambda msg: msg.data,
            rate_reducer
        )
