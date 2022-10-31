from .dispatcher_abc import DispatcherABC


class Dispatcher(DispatcherABC):
    """
    General purpose dispatcher
    Takes messages published on topic, converts them using message transformer and dispatches to all subscribers
    Supports a rate reducer
    """

    def __init__(self, ros, topic, message_transformer, rate_reducer=None):
        super().__init__()
        self.ros = ros
        self.topic = topic
        self.message_transformer = message_transformer
        self._rate_reducer = rate_reducer
        self.ros.set_listener_lambda(
            topic,
            self._message_handler
        )

    def _message_handler(self, _, message):
        if self._rate_reducer and not self._rate_reducer.should_proceed():
            return

        data = self.message_transformer(message)

        self._dispatch(data)

    def clear(self):
        self._dispatch(None)
