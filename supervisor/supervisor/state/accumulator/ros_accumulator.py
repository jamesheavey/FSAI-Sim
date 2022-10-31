from .accumulator import Accumulator


class RosAccumulator(Accumulator):
    """
    Accumulator specific to ROS messages
    Takes messages published on topic, converts them using message tranformer and send it to the underlying accumulator
    """

    def __init__(self, ros, topic, message_transformer, limit=None, rate_reducer=None):
        super().__init__(limit=limit, rate_reducer=rate_reducer)
        self.ros = ros
        self.topic = topic
        self.message_transformer = message_transformer
        self.ros.set_listener_lambda(
            topic,
            self._message_handler
        )

    def _message_handler(self, _, message):
        data = self.message_transformer(message)
        self.add_item(data)
