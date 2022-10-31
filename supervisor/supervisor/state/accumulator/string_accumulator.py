from .ros_accumulator import RosAccumulator


class StringAccumulator(RosAccumulator):
    """
    Accumulator implementation specific to string
    Create a generic accumulator, with a specific message transformer
    """

    def __init__(self, ros, topic, limit=None, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            lambda message: message.data,
            limit,
            rate_reducer
        )
