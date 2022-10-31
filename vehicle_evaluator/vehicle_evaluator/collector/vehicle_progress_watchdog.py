class VehicleProgressWatchdog:
    def __init__(self, vehicle_unmoving_duration=[], velocity_threshold=0.15, trigger_threshold=1, trigger_period=1):
        self._vehicle_unmoving_duration = vehicle_unmoving_duration
        self._trigger_threshold = trigger_threshold
        self._trigger_period = trigger_period
        self._velocity_threshold = velocity_threshold

        self._time = 0
        self._started = False
        self._last_pos = None
        self._stopped_time = None
        self._last_triggered = None

    def on_stop(self, msg):
        self._time = 0
        self._started = False
        self._last_pos = None
        self._last_triggered = None
        self._stopped_time = None

    def on_pose_update(self, vehicle_pose):
        if vehicle_pose.velocity.longitudinal > self._velocity_threshold:
            self._stopped_time = None
            self._started = True
            return

        if not self._started:
            return

        if self._stopped_time is None:
            self._stopped_time = self._time

    def on_clock(self, time):
        self._time = time

        if self._stopped_time is None:
            return

        unmoving_duration = self._time - self._stopped_time

        if unmoving_duration > self._trigger_threshold and \
                (self._last_triggered is None or self._time - self._last_triggered > self._trigger_period):
            self._last_triggered = self._time
            for callback in self._vehicle_unmoving_duration:
                callback(unmoving_duration)
