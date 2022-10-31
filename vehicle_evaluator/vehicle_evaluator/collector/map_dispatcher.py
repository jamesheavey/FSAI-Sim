from .map_handler import MapHandler


class MapDispatcher:
    def __init__(self, on_map_updated_callbacks=[]):
        self.map = None
        self._on_map_updated_callbacks = on_map_updated_callbacks

    def start(self, msg):
        self.map = MapHandler(msg.map)
        for callback in self._on_map_updated_callbacks:
            callback(self.map)

    def stop(self, msg):
        self.map = None
        for callback in self._on_map_updated_callbacks:
            callback(self.map)
