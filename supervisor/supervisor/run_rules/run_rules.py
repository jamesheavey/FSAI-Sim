from ..setup import Setup
import threading
import time


class RunRules:
    def __init__(self, ros, supervisor, simulator_manager):
        self._ros = ros
        self._supervisor = supervisor
        self._simulator_manager = simulator_manager

        topics = (
            Setup.Topics.Eval.racingline_position,
            Setup.Topics.Eval.lap,
            Setup.Topics.clock,
            Setup.Topics.penalties,
            Setup.Topics.Eval.vehicle_unmoving
        )
        for topic in topics:
            self._ros.set_listener_lambda(topic, self._handle_update)

        self._was_triggered = (False, "")
        self._thread = threading.Thread(target=self._check_triggered, daemon=True)
        self._thread.start()

    def _handle_update(self, topic, msg):
        for rule in self._supervisor.run_rule_configuration.rules:
            if rule.is_triggered(topic, msg):
                self._ros.log.info(f"Rule triggered: {rule.message}")
                self._trigger_stop(rule.message)

    def _trigger_stop(self, event):
        self._was_triggered = (True, f"Simulation stopped: {event}")

    def _check_triggered(self):
        while True:
            if self._was_triggered[0]:
                self._ros.log.info("Sending stop")
                self._ros.publish(Setup.Topics.events, self._was_triggered[1])
                self._was_triggered = (False, "")
                self._simulator_manager.stop()
            time.sleep(0.2)
