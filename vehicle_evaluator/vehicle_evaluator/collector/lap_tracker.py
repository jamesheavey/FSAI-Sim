class LapTracker:
    def __init__(self, new_lap_callbacks=[]):
        self._new_lap_callbacks = new_lap_callbacks

        self._last_ratio = 0
        self._checkpoint_count = 10
        self._checkpoints = [(i + 1) / self._checkpoint_count for i in range(self._checkpoint_count)]
        self._next_checkpoint = 0

        self._lap_counter = 0
        self._lap_start = None
        self._time = None
        self.map = None

    def set_map(self, map_handler):
        self.map = map_handler

    def _normalise_ratio(self, ratio):
        """
        Function to handle wrapping
        """
        dr = min((r - self._last_ratio for r in [ratio, ratio - 1, ratio + 1]), key=lambda r: abs(r))
        return self._last_ratio + dr

    def update_ratio(self, ratio):
        updated_ratio = self._normalise_ratio(ratio)
        next_checkpoint = self._checkpoints[self._next_checkpoint]

        # Use modulo to deal with wrapping
        if updated_ratio >= next_checkpoint and self._last_ratio < next_checkpoint:
            self._next_checkpoint += 1

        self._last_ratio = ratio

        if self._next_checkpoint < self._checkpoint_count:
            return

        self._next_checkpoint = 0
        self._new_lap()

    def update_time(self, time):
        if self._lap_start is None:
            self._lap_start = time
        self._time = time

    def reset(self, _):
        self._next_checkpoint = 0
        self._last_ratio = 0
        self._lap_counter = 0
        self._lap_start = None

    def _new_lap(self):
        for callback in self._new_lap_callbacks:
            callback(self._lap_counter, self._time - (self._lap_start or 0))
        self._lap_counter += 1
        self._lap_start = self._time


class MappedLapTracker(LapTracker):
    def pose_updated(self, vehicle_pose):
        if not vehicle_pose or not self.map:
            return
        ratio, *_ = self.map.get_closest_point_on_centerline(
            vehicle_pose.position[0:2]
        )
        self.update_ratio(ratio)
