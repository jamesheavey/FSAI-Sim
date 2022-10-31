from cv_bridge import CvBridge, CvBridgeError
from fs_utils.conversions import metadata_to_dict, quat_to_euler
from .evaluation_report import EvaluationReport
import math


class ReportAggregator:
    def __init__(self, report_started_callbacks=[], report_completed_callbacks=[]):
        self.evaluation_report = None
        self.snapshotting_resolution = 0.1
        self.last_snapshot_time = 0
        self.last_snapshot_save_time = 0
        self.reported_driver = None
        self.bridge = CvBridge()
        self._report_started_callbacks = report_started_callbacks
        self._report_completed_callbacks = report_completed_callbacks
        self.map = None
        self.car = None
        self.run_id = None

    def _report(self, key, data):
        """
        Thin wrapper around evaluation_report.add_data
        """
        if not self.evaluation_report:
            return
        self.evaluation_report.add_data(key, data)

    def _report_img(self, key, data, frame):
        """
        Thin wrapper around evaluation_report.add_image
        """
        if not self.evaluation_report:
            return
        self.evaluation_report.add_image(key, data, frame)

    def set_map(self, map):
        self.map = map
        self._try_start()

    def start(self, msg):
        self.car = msg.car
        self.run_id = msg.run_id
        self._try_start()

    def _try_start(self):
        if self.car is None or self.map is None or self.run_id is None:
            return
        self.last_snapshot_time = 0
        self.last_snapshot_save_time = 0
        self.evaluation_report = EvaluationReport(
            self.map, self.car, self.run_id, resolution=self.snapshotting_resolution)

        for callback in self._report_started_callbacks:
            callback(self.evaluation_report)

    def stop(self, msg):
        finished_report = self.evaluation_report
        self.evaluation_report = None
        finished_report.save()

        for callback in self._report_completed_callbacks:
            callback(finished_report)

        self.reported_driver = None
        self.car = None

    def report_driver(self, msg):
        if msg.driver != self.reported_driver:
            self._report("driver", msg.driver)
            self.reported_driver = msg.driver

    def report_pose(self, vehicle_pose):
        if not vehicle_pose:
            return

        self._report("position", {"x": vehicle_pose.position[0],
                                  "y": vehicle_pose.position[1],
                                  "t": vehicle_pose.rotation})
        self._report("velocity", {"x": vehicle_pose.velocity.longitudinal,
                                  "y": vehicle_pose.velocity.lateral,
                                  "w": vehicle_pose.angular_velocity})
        self._report("acceleration", {"x": vehicle_pose.acceleration.longitudinal,
                                      "y": vehicle_pose.acceleration.lateral,
                                      "a": vehicle_pose.angular_acceleration})

        if not self.map:
            return

        pos = vehicle_pose.position[0:2]

        ratio, x_centre, y_centre, dist_center = self.map.get_closest_point_on_centerline(pos)
        _, x_racing, y_racing, angle, dist_racing = self.map.get_closest_point_on_racingline(pos)
        self._report("pos_on_map", {
            "centerline": {"closest_point": {"x": x_centre, "y": y_centre}, "ratio": ratio, "distance": dist_center},
            "racingline": {"closest_point": {"x": x_racing, "y": y_racing}, "angle": angle, "distance": dist_racing}
        })

    def report_fuzzed_odom(self, odom):
        self._report("fuzzed_position", {"x": odom.pose.pose.position.x,
                                         "y": odom.pose.pose.position.y,
                                         "t": quat_to_euler(odom.pose.pose.orientation)["yaw"]})

    def report_new_lap(self, lap_count, lap_time):
        self._report("lap", {"lap_count": lap_count, "lap_time": lap_time})

    def report_raw_command(self, msg):
        def is_valid(num):
            return not math.isnan(num) and not math.isinf(num)
        linear = msg.linear.x if is_valid(msg.linear.x) else 0
        angular = msg.angular.z if is_valid(msg.angular.z) else 0
        self._report("command", {"cmd": (linear, angular)})

    def report_camera(self, topic, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        self._report_img(f"image{topic.replace('/', '_')}", {"topic": topic}, frame)

    def report_stdout(self, msg):
        self._report("stdout", {"message": msg.data})

    def report_stderr(self, msg):
        self._report("stderr", {"message": msg.data})

    def report_event(self, msg):
        self._report("simulation_event", {"message": msg.data})

    def report_metadata(self, msg):
        data = metadata_to_dict(msg)
        source = data.get("source", "")
        self._report(f"driver_metadata_{source}", {"metadata": data})

    def report_penalty(self, msg):
        self._report("penalty", msg)

    def check_snapshot(self, time):
        if time - self.last_snapshot_time > self.snapshotting_resolution:
            self.last_snapshot_time = time
            self.take_snapshot(time)

    def take_snapshot(self, time):
        """
        Store all collected data in a snapshot
        """
        if not self.evaluation_report:
            return
        self.evaluation_report.take_snapshot(time)
