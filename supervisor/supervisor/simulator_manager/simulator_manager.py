from ..setup import Setup
import time


class SimulatorManager:
    def __init__(self, ros, supervisor, map_manager):
        self._ros = ros
        self._supervisor = supervisor
        self._map_manager = map_manager
        self._clock_latch_key = "simulator_manager"
        self._timeout = 60

    def _reset(self):
        self._supervisor.reset()

    def launch(self, map, gui):
        map_msg = self._map_manager.get_map(map["map"], params=map.get("params", {}))
        ground_plane = map.get("params", {}).get("fuzzing", {}).get("groundPlane", "")
        return self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.launch_gazebo,
            map_msg,
            gui,
            ground_plane=ground_plane
        ).success

    def spawn_car(self, car):
        return self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.spawn_car,
            car
        ).success

    def launch_spawn_car(self, map, gui, car):
        self._supervisor.clock_latch.reset(self._clock_latch_key)
        launch_success = self.launch(map, gui)

        if not launch_success:
            return False

        # Wait until clock signal arrives for spawning the car
        start_time = time.time()
        while not self._supervisor.clock_latch.is_set(self._clock_latch_key) \
                and time.time() - start_time < self._timeout:
            time.sleep(0.1)

        if not self._supervisor.clock_latch.is_set(self._clock_latch_key):
            return False

        return self.spawn_car(car)

    def stop(self):
        success = self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.stop_gazebo
        ).success

        self._reset()

        return success

    def reset(self):
        resp = self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.reset_gazebo
        )
        self._reset()
        return resp

    def pause(self):
        return self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.pause_gazebo
        )

    def unpause(self):
        return self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.unpause_gazebo
        )

    def start_driver(self, drivers):
        return self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.start_driver,
            drivers
        ).success

    def stop_driver(self):
        return self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.stop_driver
        ).success

    def restart_driver(self):
        return self._ros.service_request_blocking(
            Setup.Services.SimulatorManager.restart_driver,
        ).success
