class Topic:
    """ Class to encapsulate topic description
    """

    def __init__(self, name, topic_type, message_factory=lambda: None, qos=10):
        self.name = name
        self.type = topic_type
        self.message_factory = message_factory
        self.qos = qos


class Publisher:
    """ Class to encapsulate publishing through a message factory
    """

    def __init__(self, publisher, topic):
        self._publisher = publisher
        self._message_factory = topic.message_factory

    def publish(self, *args, **kwargs):
        self._publisher.publish(
            self._message_factory(*args, **kwargs)
        )
