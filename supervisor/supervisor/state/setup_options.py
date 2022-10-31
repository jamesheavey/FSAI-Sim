from .state_abc import StateABC
from fs_utils.conversions import map_to_dict, car_to_dict
from ..setup import Setup
from .optimumlap_files import OptimumlapFiles
import time

from ament_index_python.packages import get_package_share_directory
import os


class SetupOptions(StateABC):
    """
    Class to hold all the simulation setup option information
    """

    def __init__(self, parent, ros):
        super().__init__(parent)
        self.static_maps = None
        self.generated_maps = {
            "random": "random",
            "small_track_custom": "semi_static",
            "rectangle_track": "semi_static",
            "circle_track": "semi_static"
        }
        self.cars = []
        self.drivers = []
        self.ros = ros
        self.ros.service_request(
            Setup.Services.SimulatorManager.get_maps,
            self._update_maps
        )
        self.ros.service_request(
            Setup.Services.SimulatorManager.get_cars,
            self._update_cars
        )
        self.ros.service_request(
            Setup.Services.SimulatorManager.get_drivers,
            self._update_drivers
        )

        self.optimumlap_files = OptimumlapFiles(self)

    def get_map_type(self, name):
        while self.static_maps is None:
            time.sleep(0.1)
        if name in self.static_maps:
            return "static"
        if name in self.optimumlap_files.to_dict():
            return "optimumlap"
        if name in self.generated_maps:
            return self.generated_maps[name]
        raise LookupError(f"Unknown map type: {name}")

    def _update_maps(self, maps):
        self.static_maps = list(map(lambda m: map_to_dict(m).get("name"), maps.maps))
        self.state_changed()

    def _update_cars(self, cars):
        self.cars = list(map(car_to_dict, cars.cars))
        self.state_changed()

    def get_car_preview(self, car_name):
        car = [car for car in self.cars if car["name"].lower() == car_name.lower()]
        if len(car) != 1:
            return None
        return os.path.join(get_package_share_directory("fs_description"), "pictures", car[0]["picture"])

    def _update_drivers(self, drivers):
        self.drivers = [(driver.package, driver.launch_file) for driver in drivers.drivers]
        self.state_changed()

    def to_dict(self):
        return {
            "maps": [
                *list(self.generated_maps.keys()),
                *self.optimumlap_files.to_dict(),
                *(self.static_maps or []),


            ],
            "cars": self.cars,
            "drivers": self.drivers,
            "optimumlap_files": self.optimumlap_files.to_dict()
        }
