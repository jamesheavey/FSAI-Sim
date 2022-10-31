from abc import ABC


class RateReducerABC(ABC):
    """
    Rate reducer abstract base class
    """

    def should_proceed(self):
        return True
