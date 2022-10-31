from abc import ABC


class RuleABC(ABC):
    def __init__(self, message, enabled):
        self.message = message
        self.enabled = enabled

    def is_triggered(self, source, msg):
        if not self.enabled:
            return False
        return self._is_triggered(source, msg)

    def _is_triggered(self, source, msg):
        return False
