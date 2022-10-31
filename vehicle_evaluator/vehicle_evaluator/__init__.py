from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fs_msgs.msg import GazeboStatus as GazeboStatusMsg
from fs_msgs.msg import Map, Lap, VehiclePoseOnMap, MetaData, Penalty
from fs_msgs.srv import AddReport, DeleteReport, GetReport, GetReports
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

from .util.vehicle_pose import VehiclePose
from .reports import Reports
from .collector import Republisher, StatusDispatcher, ClockDispatcher, ReportAggregator, MapDispatcher, MappedLapTracker
from .collector import PenaltyTracker, VehicleProgressWatchdog


class Setup:
    class Topic:
        cmd = "/car/cmd"
        clock = "/clock"
        tf = "/tf"
        stdout = "/driver/stdout"
        stderr = "/driver/stderr"
        metadata = "/driver/meta"
        fuzzed_odom = "/car/odom/fuzzed"

        class Cameras:
            camera1_raw = "/camera1/image_raw"
            camera1_labelled = "/camera1/image_labelled"
            camera1_fuzzed = "/camera1/image_fuzzed"
            camera2_raw = "/camera2/image_raw"
            camera2_labelled = "/camera2/image_labelled"
            camera2_fuzzed = "/camera2/image_fuzzed"
            camera0_raw = "/camera0/image_raw"
            camera0_labelled = "/camera0/image_labelled"
            camera0_fuzzed = "/camera0/image_fuzzed"
            camera_3pv = "/camera_3pv/image_raw"

        status = "/simulator_manager/gazebo_status"

        events = "/events"


class ImageSubscriptionHandler:
    def __init__(self, topic, handler):
        self._topic = topic
        self._handler = handler

    def __call__(self, msg):
        self._handler(self._topic, msg)


class VehicleEvaluatorNode(Node):
    """
    Vehicle evaluator node
    Collect as much vehicle data as possible
    Transform some, and re-publish them
    """

    def __init__(self):
        super().__init__("vehicle_evaluator")

        # Publishers
        publishers = {
            "velocity": (Twist, "/eval/velocity", 1),
            "acceleration": (Twist, "/eval/acceleration", 1),
            "racingline": (Map, "/eval/map", 1),
            "pose_on_map": (VehiclePoseOnMap, "/eval/racingline_position", 1),
            "lap": (Lap, "/eval/lap", 1),
            "report": (String, "/eval/report", 1),
            "events": (String, "/events", 1),
            "penalties": (Penalty, "/events/penalties", 1),
            "vehicle_unmoving": (Float32, "/eval/vehicle_unmoving", 1)
        }
        self.republisher = Republisher({
            key: self.create_publisher(t, topic, qos)
            for key, (t, topic, qos) in publishers.items()
        })

        self.reports = Reports(self)

        # Services
        services = (
            (GetReport, "/reports/get", self.reports.get_report),
            (GetReports, "/reports/get_all", self.reports.get_reports),
            (AddReport, "/reports/add", self.reports.add_report),
            (DeleteReport, "/reports/delete", self.reports.delete_report),
        )
        self._services = [self.create_service(t, name, handler) for t, name, handler in services]

        self.report_aggregator = ReportAggregator(
            report_started_callbacks=[
                lambda report: self.reports.set_current_report(report.id)
            ],
            report_completed_callbacks=[
                lambda _: self.reports.set_current_report(None),
                lambda report: self.republisher.publish_new_report(report.id)
            ]
        )

        self.penalty_tracker = PenaltyTracker(
               penalty_callbacks=[
                   self.republisher.publish_penalty,
                   self.republisher.publish_penalty_event,
                   self.report_aggregator.report_penalty
               ]
        )

        self.vehicle_progress_watchdog = VehicleProgressWatchdog(vehicle_unmoving_duration=[
            self.republisher.publish_vehicle_unmoving,
        ])

        self.lap_tracker = MappedLapTracker(new_lap_callbacks=[
            self.report_aggregator.report_new_lap,
            self.republisher.publish_new_lap,
            self.penalty_tracker.on_lap
        ])

        self.vehicle_pose = VehiclePose(
            pose_change_callbacks=[
                self.republisher.publish_velocity,
                self.republisher.publish_acceleration,
                self.republisher.publish_vehicle_pose_on_map,
                self.lap_tracker.pose_updated,
                self.report_aggregator.report_pose,
                self.penalty_tracker.pose_update,
                self.vehicle_progress_watchdog.on_pose_update
            ]
        )

        self.map_dispatcher = MapDispatcher(
            on_map_updated_callbacks=[
                self.republisher.publish_map,
                self.report_aggregator.set_map,
                self.lap_tracker.set_map,
                self.penalty_tracker.map_update
            ]
        )

        self.status_dispatcher = StatusDispatcher(
            update_callbacks=[
                self.report_aggregator.report_driver
            ],
            on_start_callbacks=[
                self.map_dispatcher.start,
                self.report_aggregator.start,
                self.penalty_tracker.on_start
            ],
            on_stop_callbacks=[
                self.report_aggregator.stop,
                self.lap_tracker.reset,
                self.map_dispatcher.stop,
                self.vehicle_progress_watchdog.on_stop
            ]
        )
        self.clock_dispatcher = ClockDispatcher(
            clock_callbacks=[
                self.lap_tracker.update_time,
                self.report_aggregator.check_snapshot,
                self.vehicle_progress_watchdog.on_clock
            ]
        )

        # Subscriptions
        subscriptions = (
            (GazeboStatusMsg, Setup.Topic.status, self.status_dispatcher.handle_status, 1),
            (Clock, Setup.Topic.clock, self.clock_dispatcher.handle_clock, 1),
            (TFMessage, Setup.Topic.tf, self.vehicle_pose.tf_handler, 1),
            (Twist, Setup.Topic.cmd, self.report_aggregator.report_raw_command, 1),
            (String, Setup.Topic.stdout, self.report_aggregator.report_stdout, 1),
            (String, Setup.Topic.stderr, self.report_aggregator.report_stderr, 1),
            (String, Setup.Topic.events, self.report_aggregator.report_event, 1),
            (MetaData, Setup.Topic.metadata, self.report_aggregator.report_metadata, 1),
            (Odometry, Setup.Topic.fuzzed_odom, self.report_aggregator.report_fuzzed_odom, 1),
        )
        self._subscriptions = {
            topic: self.create_subscription(t, topic, handler, qos)
            for t, topic, handler, qos in subscriptions
        }

        image_subscriptions = (
            (Setup.Topic.Cameras.camera1_raw, qos_profile_sensor_data),
            (Setup.Topic.Cameras.camera1_labelled, 1),
            (Setup.Topic.Cameras.camera1_fuzzed, qos_profile_sensor_data),
            (Setup.Topic.Cameras.camera2_raw, qos_profile_sensor_data),
            (Setup.Topic.Cameras.camera2_labelled, 1),
            (Setup.Topic.Cameras.camera2_fuzzed, qos_profile_sensor_data),
            (Setup.Topic.Cameras.camera0_raw, qos_profile_sensor_data),
            (Setup.Topic.Cameras.camera0_labelled, 1),
            (Setup.Topic.Cameras.camera0_fuzzed, qos_profile_sensor_data),
            (Setup.Topic.Cameras.camera_3pv, qos_profile_sensor_data),
        )
        for topic, qos in image_subscriptions:
            self.create_subscription(
                Image, topic, ImageSubscriptionHandler(topic, self.report_aggregator.report_camera), qos
            )
