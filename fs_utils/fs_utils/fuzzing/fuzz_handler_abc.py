from abc import ABC


class FuzzHandlerABC(ABC):
    """
    Generic fuzz handler definition
    Fuzz handler: get message, return fuzzed version
    """

    def handle(self, msg):
        return msg

    def set_config(self, config):
        pass

    def get_config(self):
        return {}

    def reset(self):
        pass
