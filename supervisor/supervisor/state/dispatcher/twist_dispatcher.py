from .dispatcher import Dispatcher


class TwistDispatcher(Dispatcher):
    """
    Dispatcher implementation specific to Twist
    Create a generic dispatcher, with a specific message transformer
    """

    def __init__(self, ros, topic, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            lambda msg: {"x": msg.linear.x, "y": msg.linear.y, "yaw": msg.angular.z},
            rate_reducer
        )
