import os
import logging
import json
from flask import Flask
from flask_socketio import SocketIO
from flask_cors import CORS
from std_msgs.msg import String, Float32
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from fs_msgs.srv import LaunchGazebo, StopGazebo, GetMaps, GetCars, SpawnCar, GetMap, GetFuzzedMap, GetSemiStaticMap
from fs_msgs.srv import GetDrivers, StartDriver, StopDriver, RestartDriver
from fs_msgs.srv import GetReport, GetReports, DeleteReport, AddReport
from fs_msgs.srv import GetFuzzingConfiguration, SetFuzzingConfiguration
from fs_msgs.msg import GazeboStatus, Map, Lap, MetaData, VehiclePoseOnMap, Penalty
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from .setup import Setup

from .ros_thread import RosThread
from .ros_node import client_factory, Topic, Service, message_factories

from . import state
from .automation import ApiRunner
from .simulator_manager import SimulatorManager
from .map_manager import MapManager
from .run_rules import RunRules

app = Flask(__name__,
            static_folder=os.path.join(os.getcwd(), 'dashboard'),
            static_url_path='/')
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")
cors = CORS(app)

logging.basicConfig(level=logging.INFO)
logging.getLogger('werkzeug').setLevel(logging.ERROR)

ros = RosThread(lambda: client_factory(
    topics=[
        Topic(Setup.Topics.demo, String, message_factories.string_message_factory),
        Topic(Setup.Topics.odometry, Odometry),
        Topic(Setup.Topics.Fuzzed.odometry, Odometry),
        Topic(Setup.Topics.SimulatorManager.gazebo_status, GazeboStatus),
        Topic(Setup.Topics.Camera1.raw, Image, qos=qos_profile_sensor_data),
        Topic(Setup.Topics.Camera1.labelled, Image),
        Topic(Setup.Topics.Camera1.fuzzed, Image, qos=qos_profile_sensor_data),
        Topic(Setup.Topics.Camera2.raw, Image, qos=qos_profile_sensor_data),
        Topic(Setup.Topics.Camera2.labelled, Image),
        Topic(Setup.Topics.Camera2.fuzzed, Image, qos=qos_profile_sensor_data),
        Topic(Setup.Topics.Camera0.raw, Image, qos=qos_profile_sensor_data),
        Topic(Setup.Topics.Camera0.labelled, Image),
        Topic(Setup.Topics.Camera0.fuzzed, Image, qos=qos_profile_sensor_data),
        Topic(Setup.Topics.Camera3PV.raw, Image, qos=qos_profile_sensor_data),
        Topic(Setup.Topics.Driver.metadata, MetaData),
        Topic(Setup.Topics.Eval.velocity, Twist),
        Topic(Setup.Topics.Eval.acceleration, Twist),
        Topic(Setup.Topics.Eval.racingline_position, VehiclePoseOnMap),
        Topic(Setup.Topics.Eval.map, Map),
        Topic(Setup.Topics.clock, Clock),
        Topic(Setup.Topics.Driver.stdout, String),
        Topic(Setup.Topics.Driver.stderr, String),
        Topic(Setup.Topics.Eval.lap, Lap),
        Topic(Setup.Topics.Eval.report, String),
        Topic(Setup.Topics.events, String, message_factories.string_message_factory),
        Topic(Setup.Topics.penalties, Penalty),
        Topic(Setup.Topics.Eval.vehicle_unmoving, Float32)
    ],
    services=[
        Service(Setup.Services.SimulatorManager.launch_gazebo, LaunchGazebo,
                message_factories.launch_gazebo_message_factory),
        Service(Setup.Services.SimulatorManager.stop_gazebo, StopGazebo,
                lambda: StopGazebo.Request()),
        Service(Setup.Services.SimulatorManager.get_maps, GetMaps,
                lambda: GetMaps.Request()),
        Service(Setup.Services.SimulatorManager.get_cars, GetCars,
                lambda: GetCars.Request()),
        Service(Setup.Services.SimulatorManager.spawn_car, SpawnCar,
                message_factories.spawn_car_message_factory),
        Service(Setup.Services.SimulatorManager.pause_gazebo, Empty,
                message_factories.empty_message_factory),
        Service(Setup.Services.SimulatorManager.unpause_gazebo, Empty,
                message_factories.empty_message_factory),
        Service(Setup.Services.SimulatorManager.reset_gazebo, Empty,
                message_factories.empty_message_factory),
        Service(Setup.Services.SimulatorManager.get_drivers, GetDrivers,
                lambda: GetDrivers.Request()),
        Service(Setup.Services.SimulatorManager.start_driver, StartDriver,
                message_factories.start_driver_message_factory),
        Service(Setup.Services.SimulatorManager.stop_driver, StopDriver,
                lambda: StopDriver.Request()),
        Service(Setup.Services.SimulatorManager.restart_driver, RestartDriver,
                lambda: RestartDriver.Request()),
        Service(Setup.Services.SimulatorManager.get_static_map, GetMap,
                lambda m, track_width=0: GetMap.Request(map=m, track_width=float(track_width))),
        Service(Setup.Services.SimulatorManager.get_optimumlap_map, GetMap,
                lambda m, track_width=0: GetMap.Request(map=m, track_width=float(track_width))),
        Service(Setup.Services.SimulatorManager.get_random_map, GetMap,
                lambda m, track_width=0: GetMap.Request(map=json.dumps(m), track_width=float(track_width))),
        Service(Setup.Services.VehicleEvaluator.get_report, GetReport,
                lambda idx: GetReport.Request(id=str(idx))),
        Service(Setup.Services.VehicleEvaluator.get_reports, GetReports,
                lambda: GetReports.Request()),
        Service(Setup.Services.VehicleEvaluator.add_report, AddReport,
                lambda zip_path: AddReport.Request(zip_path=str(zip_path))),
        Service(Setup.Services.VehicleEvaluator.delete_report, DeleteReport,
                lambda idx: DeleteReport.Request(id=str(idx))),
        Service(Setup.Services.Fuzzing.get_configuration, GetFuzzingConfiguration,
                lambda: GetFuzzingConfiguration.Request()),
        Service(Setup.Services.Fuzzing.set_configuration, SetFuzzingConfiguration,
                lambda config: SetFuzzingConfiguration.Request(config_json=json.dumps(config))),
        Service(Setup.Services.Fuzzing.reset, Empty,
                message_factories.empty_message_factory),
        Service(Setup.Services.SimulatorManager.get_fuzzed_map, GetFuzzedMap,
                lambda map, seed, cone_misplaced, cone_missing: GetFuzzedMap.Request(
                    map=map, seed=int(seed), cone_misplaced=float(cone_misplaced), cone_missing=float(cone_missing))),
        Service(Setup.Services.SimulatorManager.get_semi_static_map, GetSemiStaticMap,
                message_factories.get_semi_static_map)
    ]
))

supervisor_state = state.Supervisor(ros)
map_manager = MapManager(ros, supervisor_state)
simulator_manager = SimulatorManager(ros, supervisor_state, map_manager)
run_rules = RunRules(ros, supervisor_state, simulator_manager)
api_runner = ApiRunner(supervisor_state, simulator_manager)

from .api import *          # noqa: E402, F403, F401
from .socketio import *     # noqa: E402, F403, F401
