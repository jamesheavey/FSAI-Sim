from abc import ABC


class DispatcherABC(ABC):
    """
    Base dispatcher class only handling dispatching and subscription, including decorator
    This is a push-in, push-out data structure
    """

    def __init__(self):
        self._handlers = set()

    def on_update(self, handler):       # Decorator
        self.subscribe(handler)
        return handler

    def subscribe(self, callback):
        self._handlers.add(callback)

    def _dispatch(self, *args, **kwargs):
        for handler in self._handlers:
            handler(*args, **kwargs)
