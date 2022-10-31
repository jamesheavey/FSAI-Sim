from .rate_reducer_abc import RateReducerABC


class CounterRateReducer(RateReducerABC):
    """
    Rate reducer implementation that reduces the rate by a whole number
    E.g.: Only allow every 2nd, 3rd, etc calls
    """

    def __init__(self, rate_reduction):
        assert rate_reduction > 0 and type(rate_reduction) == int
        self._rate_reduction = rate_reduction
        self._rate_counter = 0

    def should_proceed(self):
        self._rate_counter += 1
        return self._rate_counter % self._rate_reduction == 0
