

class ApiRunner:
    def __init__(self, supervisor, simulator_manager):
        self._supervisor = supervisor
        self._simulator_manager = simulator_manager

    def run(self, request):
        if self._currently_running():
            raise Exception("Simulator already running")

        fuzzing = request.get("fuzzing", {})
        self._supervisor.fuzz_controller.set_configuration(fuzzing)

        rules = request.get("rules", {})
        self._supervisor.run_rule_configuration.set_configuration(rules)

        run = request.get("run", {})

        map = run.get("map", None)
        if map is None:
            raise Exception("Missing property 'map'")
        map_id = map.get("id", None)
        if map_id is None:
            raise Exception("map is missing property 'id'")
        map_parameters = map.get("parameters", {})

        car = run.get("car", None)
        if car is None:
            raise Exception("Missing property 'car'")
        car_id = car.get("id", None)
        if car_id is None:
            raise Exception("car is missing property 'id'")

        drivers = run.get("driver", [])
        if len(drivers) <= 0:
            raise Exception("Missing property 'driver'")

        if any("package" not in driver or "launch_file" not in driver for driver in drivers):
            raise Exception("Malformed driver request")

        launch_result = self._simulator_manager.launch_spawn_car(
            {"map": map_id, "params": map_parameters}, False, car_id)

        if not launch_result:
            raise Exception("Failed to launch map")

        driver_result = self._simulator_manager.start_driver(drivers)

        if not driver_result:
            raise Exception("Failed to start driver")

        return self.get_configuration()

    def get_options(self):
        options = self._supervisor.options.to_dict()
        return {
            "run": {
                "cars": [{"id": car["name"]} for car in options["cars"]],
                "drivers": [{"package": pkg, "launch_file": launch} for pkg, launch in options["drivers"]],
                "maps": [{"id": m} for m in options["maps"]]
            }
        }

    def get_configuration(self):
        return {
            "fuzzing": self._supervisor.fuzz_controller.to_dict(),
            "rules": self._supervisor.run_rule_configuration.to_dict(),
            "current_run": {
                key: value if key != "map" else value.get("name", "")
                for key, value in self._supervisor.current_run.to_dict().items()
                if key not in ["eval_map", "was_reset"]
            }
        }

    def _currently_running(self):
        return self._supervisor.current_run.ready

    def stop(self):
        if not self._currently_running():
            raise Exception("Simulator not running")

        res = self._simulator_manager.stop()
        if not res:
            raise Exception("Failed to stop simulator")

        return {"success": True}
