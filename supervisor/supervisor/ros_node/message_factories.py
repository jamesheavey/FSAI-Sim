from std_msgs.msg import String
from std_srvs.srv import Empty
from example_interfaces.srv import AddTwoInts
from fs_msgs.msg import Driver
from fs_msgs.srv import LaunchGazebo, SpawnCar, StartDriver, GetSemiStaticMap  # , StopGazebo, SpawnCar
import json


def string_message_factory(data):
    return String(data=data)


def add_two_ints_message_factory(a, b):
    req = AddTwoInts.Request()
    req.a = a
    req.b = b
    return req


def launch_gazebo_message_factory(map_msg, gui, timeout=0, ground_plane="nothing"):
    req = LaunchGazebo.Request()
    req.map = map_msg
    req.gui = gui
    req.timeout = timeout
    req.ground_plane = ground_plane
    return req


def spawn_car_message_factory(car_name):
    req = SpawnCar.Request()
    req.car_name = car_name
    return req


def start_driver_message_factory(drivers):
    req = StartDriver.Request()
    req.drivers = [
        Driver(
            package=driver["package"],
            launch_file=driver["launch_file"],
            parameters=json.dumps(driver.get("parameters", {}))
        ) for driver in drivers]
    return req


def empty_message_factory():
    return Empty.Request()


def get_semi_static_map(map_name, track_width=3, radius=10, length=60, height=60, cone_separation_distance=2, **kwargs):
    return GetSemiStaticMap.Request(
        map=map_name,
        track_width=float(track_width),
        radius=float(radius),
        length=float(length),
        height=float(height),
        cone_separation_distance=float(cone_separation_distance)
    )
