class StatusDispatcher:
    def __init__(self,
                 status_changed_callbacks=[],
                 update_callbacks=[],
                 on_start_callbacks=[],
                 on_stop_callbacks=[]):
        self._status_changed_callbacks = status_changed_callbacks
        self._update_callbacks = update_callbacks
        self._on_start_callbacks = on_start_callbacks
        self._on_stop_callbacks = on_stop_callbacks
        self._sim_status = False

    def _get_sim_status(self, msg):
        """
        Get status based on status message
        return true: running; false: not running
        """
        return (msg.status == msg.STATUS_HEALTHY) and bool(msg.car.name) and not msg.was_reset

    def handle_status(self, msg):
        for callback in self._update_callbacks:
            callback(msg)

        new_status = self._get_sim_status(msg)
        if new_status == self._sim_status:
            return

        for callback in self._status_changed_callbacks:
            callback(new_status, msg)

        for callback in (self._on_start_callbacks if new_status else self._on_stop_callbacks):
            callback(msg)

        self._sim_status = new_status
