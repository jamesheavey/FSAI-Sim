import os
import xacro
from fs_msgs.msg import Car as CarMsg

from ament_index_python.packages import get_package_share_directory


class Car:
    """
    Simple encapsulation of car information
    """

    def __init__(self, name, path, picture, width=1.76, length=4.575):       # Defaults for the Prius
        self.name = name
        self.path = path
        self.width = width
        self.length = length
        self.picture = picture

    def to_msg(self):
        return CarMsg(
            name=self.name,
            width=self.width,
            length=self.length,
            picture=self.picture
        )


class CarProvider:
    # TODO: don't use an /src directory, use a /tmp directory
    car_dir = "/home/ros/fs_ws/src/f21ai-sim/simulator_manager/vehicles"

    cars = {
        "flat_car": Car("flat_car", "flat_car.urdf.xacro", "Flat_car.png", width=1.245, length=1.753),
        "FS_car": Car("FS_car", "FS_Vehicle_Mesh.urdf.xacro", "FS_Car.png", width=1.356, length=2.530),
        "IMeche_car": Car(
            "IMeche_car", "IMeche_car/IMeche_Vehicle_Mesh.urdf.xacro", "IMeche_Car.png", width=1.430, length=2.8146),
    }
    default_car = "flat_car"

    @staticmethod
    def get_car_options():
        return list(CarProvider.cars.values())

    @staticmethod
    def get_car(car_name):
        if car_name not in CarProvider.cars:
            car_name = CarProvider.default_car

        car = CarProvider.cars.get(car_name)
        # Find absolute path
        path = os.path.join(
            get_package_share_directory("fs_description"),
            "robots", car.path
        )

        if not os.path.isfile(path):
            raise FileNotFoundError(path)

        # If car is defined as an xacro, process the xacro
        if "xacro" in path:
            return car, CarProvider.process_xacro(path)
        return car, path

    @staticmethod
    def process_xacro(file):
        car_path = os.path.join(CarProvider.car_dir, "robot.urdf")
        doc = xacro.process_file(file)
        out = xacro.open_output(car_path)
        out.write(doc.toprettyxml(indent="  "))
        return car_path
