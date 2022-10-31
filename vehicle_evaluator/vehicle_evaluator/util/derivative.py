class Derivative:
    """
    Calculate the derivative of a single value.
    It holds the previous value and time
    """
    def __init__(self, initial_value=None, initial_time=0, value_range=None):
        self._value = initial_value
        self._time = initial_time
        self._last_value = 0
        self._value_range = value_range

    def _dv(self, value):
        """
        if value_range is specified, then values can wrap around. This supports that
        """
        if self._value_range is None:
            return value - self._value

        def scale(v):
            return (v - self._value_range[0]) / (self._value_range[1] - self._value_range[0])
        scaled_value = scale(value)
        scaled__value = scale(self._value)

        offset = min([-1, 0, 1], key=lambda o: abs(scaled_value + o - scaled__value))
        scaled_dv = scaled_value + offset - scaled__value
        return scaled_dv * (self._value_range[1] - self._value_range[0])

    def update(self, value, time):
        """
        update derivative
        """
        if self._value is None:
            self._value = value
            self._time = time
            return 0
        if time == self._time:
            return self._last_value

        dv = self._dv(value)
        dt = time - self._time
        self._value = value
        self._time = time
        self._last_value = dv / dt
        return self._last_value

    @property
    def value(self):
        return self._last_value
