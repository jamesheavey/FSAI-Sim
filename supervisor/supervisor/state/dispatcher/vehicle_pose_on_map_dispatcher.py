from .dispatcher import Dispatcher


class VehiclePoseOnMapDispatcher(Dispatcher):
    """
    Dispatcher implementation specific to driver metadata
    Create a generic dispatcher, with a specific message transformer
    """

    def __init__(self, ros, topic, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            VehiclePoseOnMapDispatcher.vehicle_pose_on_map_message_transformer,
            rate_reducer
        )

    @staticmethod
    def vehicle_pose_on_map_message_transformer(msg):
        return {
            "position_on_racingline": {"x": msg.position_on_racingline.x, "y": msg.position_on_racingline.y},
            "position_on_centerline": {"x": msg.position_on_centerline.x, "y": msg.position_on_centerline.y},
            "racingline_angle": msg.racingline_angle,
            "centerline_ratio": msg.centerline_ratio,
            "distance": msg.distance,
            "track_width": msg.track_width
        }
