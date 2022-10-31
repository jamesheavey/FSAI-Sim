from collections import deque


class WindowedAverage:
    """
    Simple fixed window average using deque
    """
    def __init__(self, size):
        self._q = deque([], maxlen=size)

    def update(self, value):
        self._q.append(value)
        return sum(self._q) / len(self._q)

    @property
    def value(self):
        if len(self._q) <= 0:
            return 0
        return sum(self._q) / len(self._q)
