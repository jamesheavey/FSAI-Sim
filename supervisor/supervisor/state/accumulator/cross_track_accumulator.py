from .ros_accumulator import RosAccumulator


class RmsCrossTrackAccumulator(RosAccumulator):
    """
    Accumulator implementation specific to extracting distance from centerline and calculating RMS CTE
    Create a generic accumulator, with a specific message transformer
    """

    def __init__(self, ros, topic, limit=None, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            lambda message: message.distance ** 2,
            limit,
            rate_reducer
        )
