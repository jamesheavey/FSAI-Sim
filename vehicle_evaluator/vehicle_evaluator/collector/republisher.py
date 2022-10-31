from geometry_msgs.msg import Twist, Vector3
from fs_msgs.msg import Lap, VehiclePoseOnMap, Penalty
import numpy as np
from std_msgs.msg import String, Float32


class Republisher:
    def __init__(self, publishers={}):
        self._publishers = publishers
        self._map = None

    def publish_velocity(self, vehicle_pose):
        publisher = self._publishers.get("velocity", None)
        if not publisher or not vehicle_pose:
            return
        msg = Twist()
        msg.linear.x = vehicle_pose.velocity.longitudinal
        msg.linear.y = vehicle_pose.velocity.lateral
        msg.angular.z = vehicle_pose.angular_velocity
        publisher.publish(msg)

    def publish_acceleration(self, vehicle_pose):
        publisher = self._publishers.get("acceleration", None)
        if not publisher or not vehicle_pose:
            return
        msg = Twist()
        msg.linear.x = vehicle_pose.acceleration.longitudinal
        msg.linear.y = vehicle_pose.acceleration.lateral
        msg.angular.z = vehicle_pose.angular_acceleration
        publisher.publish(msg)

    def publish_map(self, map_handler):
        self._map = map_handler
        publisher = self._publishers.get("racingline", None)
        if not publisher or not map_handler:
            return
        publisher.publish(map_handler.map_msg)

    def publish_vehicle_pose_on_map(self, vehicle_pose):
        publisher = self._publishers.get("pose_on_map", None)
        if not publisher or not vehicle_pose or not self._map:
            return

        position = vehicle_pose.position[0:2]
        _, rx, ry, angle, _ = self._map.get_closest_point_on_racingline(position)
        ratio, cx, cy, dist = self._map.get_closest_point_on_centerline(position)
        width = self._map.get_map_width_at_ratio(ratio)
        msg = VehiclePoseOnMap(
            position_on_racingline=Vector3(x=rx, y=ry),
            position_on_centerline=Vector3(x=cx, y=cy),
            racingline_angle=float(angle),
            centerline_ratio=float(ratio),
            distance=float(dist),
            track_width=float(width),
            car_angle_to_racingline_angle=float((vehicle_pose.rotation - angle + np.pi) % (2 * np.pi) - np.pi)
        )
        publisher.publish(msg)

    def publish_new_lap(self, lap_count, lap_time):
        publisher = self._publishers.get("lap", None)
        if not publisher:
            return
        publisher.publish(Lap(lap_count=lap_count, lap_time=lap_time))

    def publish_new_report(self, report):
        publisher = self._publishers.get("report", None)
        if not publisher:
            return
        publisher.publish(String(data=str(report)))

    def publish_penalty(self, penalty):
        publisher = self._publishers.get("penalties", None)
        if not publisher:
            return
        publisher.publish(
            Penalty(name=str(penalty["name"]), lap=int(penalty["lap"]), penalty_time=float(penalty['penalty_time']))
        )

    def publish_penalty_event(self, penalty):
        publisher = self._publishers.get("events", None)
        if not publisher:
            return
        publisher.publish(
            String(data=f"Penalty: {penalty['name']}; {penalty['penalty_time']}s; Lap {penalty['lap']}")
        )

    def publish_vehicle_unmoving(self, duration):
        publisher = self._publishers.get("vehicle_unmoving", None)
        if not publisher:
            return

        publisher.publish(Float32(data=float(duration)))
