from .dispatcher import Dispatcher
from fs_utils.conversions import map_to_dict


class DriverMetadataDispatcher(Dispatcher):
    """
    Dispatcher implementation specific to driver metadata
    Create a generic dispatcher, with a specific message transformer
    """

    def __init__(self, ros, topic, rate_reducer=None):
        super().__init__(
            ros,
            topic,
            DriverMetadataDispatcher.driver_metadata_message_transformer,
            rate_reducer
        )

    @staticmethod
    def driver_metadata_message_transformer(msg):
        return {
            "timestamp": msg.timestamp,
            "steer": msg.steer,
            "acceleration": msg.acceleration,
            "map": map_to_dict(msg.map)
        }
