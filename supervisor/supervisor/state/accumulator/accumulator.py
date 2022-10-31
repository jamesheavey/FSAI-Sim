from collections import deque
from ..dispatcher import DispatcherABC


class Accumulator(DispatcherABC):
    """
    General purpose accumulator
    Notifies subscribers of new messages or when the queue is cleared
    Storage can be limited to a specific number of messages
    Supports a rate reducer
    """

    def __init__(self, limit=None, rate_reducer=None):
        super().__init__()
        self._rate_reducer = rate_reducer
        self._accum = deque([], limit)

    def add_item(self, item):
        if self._rate_reducer and not self._rate_reducer.should_proceed():
            return

        self._accum.append(item)
        self._dispatch("append", item)

    def get_data(self):
        return list(self._accum)

    def clear(self):
        self._accum.clear()
        self._dispatch("clear", [])
