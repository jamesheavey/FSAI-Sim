from abc import ABC


class StateABC(ABC):
    """
    State abstract base class
    State holds some information;
    Notifies patrent (usually also a state) if some information has changed
    Can be explicitly converted to a dictionary
    """

    def __init__(self, parent):
        self._parent = parent

    def state_changed(self):
        self._parent.state_changed()

    def to_dict(self):
        return {}
