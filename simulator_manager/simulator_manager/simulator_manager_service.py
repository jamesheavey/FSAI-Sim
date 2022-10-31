from rclpy.node import Node
import time
from fs_msgs.srv import LaunchGazebo, StopGazebo, SpawnCar, GetMaps, GetCars
from fs_msgs.srv import GetDrivers, StartDriver, StopDriver, RestartDriver, GetMap, GetSemiStaticMap
from fs_msgs.srv import GetFuzzedMap
from fs_msgs.msg import GazeboStatus as GazeboStatusMsg
from fs_msgs.msg import Car as CarMsg
from fs_msgs.msg import Driver as DriverMsg
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from std_msgs.msg import String

from .gazebo_manager import GazeboManager
from .map_provider import OptimumLapMap, StaticMaps, GenerateMap, RandomMap, MapSize, MapComplexity
from .map_provider import ConeMissing, ConeMisplacement, ModifyMap, GroundPlanes, Lighting, SemiStaticMaps
from .car_provider import CarProvider
from .car_spawner import spawn_car
from .driver_manager import DriverManager
from uuid import uuid1
import json

import traceback


def gen_id(segments=1):
    return ''.join(str(uuid1()).split("-")[:segments])


class SimulatorManagerService(Node):

    default_track_width = 3.0

    def __init__(self):
        super().__init__("Simulator_manager_service")

        services = [
            (GetMap, "/simulator_manager/get_static_map", self.get_static_map_handler),
            (GetMap, "/simulator_manager/get_optimumlap_map", self.get_optimumlap_map_handler),
            (GetMap, "/simulator_manager/get_random_map", self.get_random_map_handler),
            (GetSemiStaticMap, "/simulator_manager/get_semi_static_map", self.get_semi_static_map_handler),
            (GetFuzzedMap, "/simulator_manager/get_fuzzed_map", self.get_fuzzed_map_handler),
            (LaunchGazebo, "/simulator_manager/launch_gazebo", self.launch_map_handler),
            (StopGazebo, "/simulator_manager/stop_gazebo", self.shutdown_map_handler),
            (Empty, "/simulator_manager/pause_gazebo", self.pause_map_handler),
            (Empty, "/simulator_manager/unpause_gazebo", self.unpause_map_handler),
            (Empty, "/simulator_manager/reset_gazebo", self.reset_map_handler),
            (GetMaps, "/simulator_manager/get_maps", self.get_maps_handler),
            (SpawnCar, "/simulator_manager/spawn_car", self.spawn_car_handler),
            (GetCars, "/simulator_manager/get_cars", self.get_cars_handler),
            (GetDrivers, "/simulator_manager/get_drivers", self.get_drivers_handler),
            (StartDriver, "/simulator_manager/start_driver", self.start_driver_handler),
            (StopDriver, "/simulator_manager/stop_driver", self.stop_driver_handler),
            (RestartDriver, "/simulator_manager/restart_driver", self.restart_driver_handler),
        ]
        self._services = [self.create_service(t, name, handler) for t, name, handler in services]

        self.pause_client = self.create_client(
            Empty, "/pause_physics"
        )

        self.unpause_client = self.create_client(
            Empty, "/unpause_physics"
        )

        self.reset_client = self.create_client(
            Empty, "/reset_simulation"
        )

        self.status_publisher = self.create_publisher(
            GazeboStatusMsg, "/simulator_manager/gazebo_status", 1
        )

        self.driver_stdout_publisher = self.create_publisher(
            String, "/driver/stdout", 1
        )

        self.driver_stderr_publisher = self.create_publisher(
            String, "/driver/stderr", 1
        )

        self.clock_received = False
        self.clock_subscriber = self.create_subscription(
            Clock, "/clock", self.clock_callback, 1
        )

        self.status_timer = self.create_timer(0.5, self.check_status)
        self._last_status = None

        self.gazebo_manager = GazeboManager()
        self.gazebo_manager.killall()

        self.current_car = None
        self.was_reset = False
        self.run_id = ""

        self.driver_manager = DriverManager(self._driver_stdout_callback, self._driver_stderr_callback)

    def get_semi_static_map_handler(self, request, response):
        self.get_logger().info(f"Getting map: {request}")
        try:
            response.map = SemiStaticMaps.get_map(
                request.map,
                request.track_width or SimulatorManagerService.default_track_width,
                request.radius,
                request.length,
                request.height,
                cone_separation_distance=2).to_msg()
        except Exception:
            response.map = None
        if response.map is None:
            response.success = False
        response.success = True
        return response

    def get_fuzzed_map_handler(self, request, response):
        self.get_logger().info(f"Fuzzing map: {request.seed} {request.cone_misplaced} {request.cone_missing}")
        fuzz_seed = int(time.time())
        response.map = ModifyMap.get_fuzzed_map(
            request.seed or fuzz_seed,
            request.map,
            request.cone_misplaced or ConeMisplacement.medium,
            request.cone_missing or ConeMissing.medium)
        response.success = True
        return response

    def get_random_map_handler(self, request, response):
        self.get_logger().info(f"Getting map: {request}")
        # Get map and seed used to generate the map
        map_seed = int(time.time())
        try:
            params = json.loads(request.map)
        except json.JSONDecodeError:
            params = {}

        response.map = RandomMap.get_map(
            params,
            request.track_width or SimulatorManagerService.default_track_width,
            params.get("seed", map_seed),
            params.get("size", MapSize.medium),  # default map size
            params.get("complexity", MapComplexity.medium),  # default map complexity
            allow_intersections=params.get("allow_intersections", False),
            cone_separation_distance=2
        ).to_msg()
        response.success = True
        return response

    def get_optimumlap_map_handler(self, request, response):
        self.get_logger().info(f"Getting map: {request}")
        response.map = OptimumLapMap.get_map(
            request.map, request.track_width or SimulatorManagerService.default_track_width,
            cone_separation_distance=2).to_msg()
        response.success = True
        return response

    def get_static_map_handler(self, request, response):
        self.get_logger().info(f"Getting map: {request}")
        response.map = StaticMaps.generate_static_map(
            request.map, request.track_width or SimulatorManagerService.default_track_width,
            cone_separation_distance=2).to_msg()
        response.success = True
        return response

    def clock_callback(self, _):
        self.clock_received = True

    def check_status(self):
        running = self.clock_received
        self.clock_received = False

        status = self.gazebo_manager.current_status
        driver = self.driver_manager.signature
        new_status = (status, self.current_car, running, self.was_reset, driver, self.run_id)

        if self._last_status is None:
            self._last_status = status
            return
        if new_status == self._last_status:
            return
        self._last_status = new_status

        current_map = self.gazebo_manager.current_map_msg
        current_car = self.current_car.to_msg() if self.current_car is not None else CarMsg()
        self.status_publisher.publish(GazeboStatusMsg(
            status=status,
            map=current_map,
            car=current_car,
            running=running,
            was_reset=self.was_reset,
            driver=driver,
            run_id=self.run_id
        ))
        self.was_reset = False

    def force_check_status(self):
        cr = self.clock_received
        self.check_status()
        self.clock_received = cr    # Persist clock_received through out of time check

    def launch_map_handler(self, request, response):
        self.get_logger().info("Launching map: {}, gui = {}, timeout = {}, ground plane = {}".format(
            request.map.name,
            request.gui,
            request.timeout,
            request.ground_plane
        ))
        try:
            selected_map = GenerateMap.from_msg(
                request.map,
                request.ground_plane or GroundPlanes.nothing,
                request.lighting or Lighting.clear_day)  # convert to map
            self.gazebo_manager.launch(
                selected_map,
                request.gui,
                request.timeout)
        except Exception as e:
            self.get_logger().error(f"Failed to launch map: {e}\n{traceback.format_exc()}")
            response.success = False
        else:
            response.success = True
            response.map = selected_map.to_msg()
            self.current_car = None
            self.run_id = gen_id()
        self.force_check_status()
        return response

    def shutdown_map_handler(self, request, response):
        self.get_logger().info("Stopping")

        try:
            self.gazebo_manager.shutdown()
            self.driver_manager.stop_driver_if_running()
        except Exception as e:
            self.get_logger().error(f"Failed to stop map: {e}\n{traceback.format_exc()}")
            response.success = False
        else:
            response.success = True
            self.get_logger().info("Stopped")
            self.current_car = None
            self.run_id = ""
        self.force_check_status()
        return response

    def pause_map_handler(self, request, response):
        def future_handler(result):
            self.clock_received = False
            self.force_check_status()

        self.get_logger().info("Paused")
        fut = self.pause_client.call_async(Empty.Request())
        fut.add_done_callback(future_handler)

        return response

    def unpause_map_handler(self, request, response):
        def future_handler(result):
            self.clock_received = True
            self.force_check_status()

        self.get_logger().info("Unpaused")
        fut = self.unpause_client.call_async(Empty.Request())
        fut.add_done_callback(future_handler)

        return response

    def reset_map_handler(self, request, response):
        def future_handler(result):
            self.run_id = gen_id()
            self.force_check_status()     # This emmits was reset=False
            try:
                self.driver_manager.restart_drivers()
            except Exception as e:
                self.get_logger().error(f"Failed to restart drivers: {e}\n{traceback.format_exc()}")

        self.get_logger().info("Reset")
        self.was_reset = True
        self.force_check_status()     # This emmits was reset=True
        fut = self.reset_client.call_async(Empty.Request())
        fut.add_done_callback(future_handler)

        return response

    def get_maps_handler(self, request, response):
        response.maps = list(map(lambda m: m.to_msg(),
                                 StaticMaps.get_maps()))
        return response

    def spawn_car_handler(self, request, response):
        self.get_logger().info(f"Spawning car: {request}")

        if self.current_car is not None:
            response.success = False
            return response

        current_map = self.gazebo_manager.current_map

        try:
            self.current_car, path = CarProvider.get_car(request.car_name)
            spawn_car(path,
                      # TODO: use pose from request
                      **current_map.start.get("position", {}),
                      **current_map.start.get("rotation", {})
                      )
        except Exception as e:
            self.get_logger().error(f"Failed to launch car: {e}\n{traceback.format_exc()}")
            response.success = False
        else:
            response.success = True
        self.force_check_status()
        return response

    def get_cars_handler(self, request, response):
        response.cars = list(map(lambda m: m.to_msg(),
                                 CarProvider.get_car_options()))
        return response

    def get_drivers_handler(self, request, response):
        response.drivers = [
            DriverMsg(package=package, launch_file=launch_file)
            for package, launch_file in self.driver_manager.get_possible_drivers()
        ]
        return response

    def start_driver_handler(self, request, response):
        try:
            self.driver_manager.launch_drivers(request.drivers)
        except Exception as e:
            self.get_logger().error(f"Failed to start drivers: {e}\n{traceback.format_exc()}")
            response.success = False
        else:
            response.success = True
        self.force_check_status()
        return response

    def stop_driver_handler(self, request, response):
        try:
            self.driver_manager.stop_drivers()
        except Exception as e:
            self.get_logger().error(f"Failed to stop drivers: {e}\n{traceback.format_exc()}")
            response.success = False
        else:
            response.success = True
        self.force_check_status()
        return response

    def restart_driver_handler(self, request, response):
        try:
            self.driver_manager.restart_drivers()
        except Exception as e:
            self.get_logger().error(f"Failed to restart drivers: {e}\n{traceback.format_exc()}")
            response.success = False
        else:
            response.success = True
        self.force_check_status()
        return response

    def _driver_stdout_callback(self, line):
        self.driver_stdout_publisher.publish(String(data=line))

    def _driver_stderr_callback(self, line):
        self.driver_stderr_publisher.publish(String(data=line))
