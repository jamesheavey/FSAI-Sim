from .dispatcher_abc import DispatcherABC
from fs_utils.conversions import metadata_to_dict


class MetadataDispatcher(DispatcherABC):
    """
    Dispatcher specifically designed to support metadata messages from multiple sources on the same topic
    Supports a rate reducer
    """

    def __init__(self, ros, topic, rate_reducer=None):
        super().__init__()
        self.ros = ros
        self.topic = topic
        self._rate_reducer = rate_reducer
        self.ros.set_listener_lambda(
            topic,
            self._message_handler
        )
        self._known_sources = set()

    def _message_handler(self, _, message):
        if self._rate_reducer and not self._rate_reducer.should_proceed():
            return

        data = metadata_to_dict(message)
        source = data.get("source", "")

        self._known_sources.add(source)

        self._dispatch(data, source=source)

    def clear(self):
        for source in self._known_sources:
            self._dispatch(None, source=source)
