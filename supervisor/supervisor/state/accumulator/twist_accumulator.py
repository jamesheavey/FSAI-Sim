from .ros_accumulator import RosAccumulator


class TwistAccumulator(RosAccumulator):
    """
    Accumulator implementation specific to Twist
    Create a generic accumulator, with a specific message transformer
    """

    def __init__(self, ros, topic, limit=None, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            lambda msg: {"x": msg.linear.x, "y": msg.linear.y, "yaw": msg.angular.z},
            limit,
            rate_reducer
        )
