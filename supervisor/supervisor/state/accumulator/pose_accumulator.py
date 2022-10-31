from .ros_accumulator import RosAccumulator
from fs_utils.conversions import pose_to_dict


class PoseAccumulator(RosAccumulator):
    """
    Accumulator implementation specific to Pose
    Create a generic accumulator, with a specific message transformer
    """

    def __init__(self, ros, topic, limit=None, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            lambda message: pose_to_dict(message.pose.pose),
            limit,
            rate_reducer
        )
