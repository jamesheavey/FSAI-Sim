from .ros_accumulator import RosAccumulator


class PenaltyAccumulator(RosAccumulator):
    """
    Accumulator implementation specific to Penalty
    Create a generic accumulator, with a specific message transformer
    """

    def __init__(self, ros, topic, limit=None, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            lambda msg: {"name": msg.name, "lap": msg.lap, "penalty_time": msg.penalty_time},
            limit,
            rate_reducer
        )
