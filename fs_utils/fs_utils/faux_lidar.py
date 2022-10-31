from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from fs_msgs.msg import GazeboStatus
from rosgraph_msgs.msg import Clock
import rclpy
import math
import numpy as np
from fs_utils.conversions import quat_to_euler

TOPIC = "/lidar"
HALF_FOV = math.pi * 2 / 3
LASER_RESOLUTION = 128
TIME_RESOLUTION = 0.2
RANGE_MIN = 0.3
RANGE_MAX = 15
CONE_RADIUS = 0.2


class FauxLidar(Node):
    """
    This node pretends to be lidar.
    It takes the map from the simulator and the position of the car, and calculates what a lidar output would be
    It does not account for cones that move.
    Once the driver supports actual lidar, stop using it. It's quite resource intensive
    """

    def __init__(self):
        super().__init__("faux_lidar")

        self.publisher = self.create_publisher(LaserScan, TOPIC, 1)

        self.angle_min = -HALF_FOV
        self.angle_max = HALF_FOV
        self.angle_increment = 2 * HALF_FOV / LASER_RESOLUTION
        self.time_increment = 0
        self.scan_time = TIME_RESOLUTION
        self.range_min = RANGE_MIN
        self.range_max = RANGE_MAX

        self.create_subscription(Odometry, "/car/odom", self._odom_handler, 1)
        self.create_subscription(GazeboStatus, "/simulator_manager/gazebo_status", self._status_handler, 1)
        self.create_subscription(Clock, "/clock", self._clock_handler, 1)

        self._cones = None
        self._position = None
        self._last_scan_time = 0

    def _clock_handler(self, msg):
        """
        Handle clock update
        """
        time = msg.clock.sec + msg.clock.nanosec / 1e9
        # Dispatch new scan
        if time < self._last_scan_time or time - self._last_scan_time > self.scan_time:
            self._scan()
            self._last_scan_time = time

    def _status_handler(self, msg):
        """
        Handle gazebo status update
        """
        if msg.status == msg.STATUS_HEALTHY and msg.car.name != "":
            # Store cone positions
            self._cones = [(pose.position.x, pose.position.y)
                           for pose in msg.map.blue_cones + msg.map.yellow_cones + msg.map.orange_cones]
        # Else: Reset variables
        else:
            self._cones = None
            self._position = None
            self._last_scan_time = 0

    def _odom_handler(self, msg):
        """
        Handle odometry updates
        """
        self._position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_euler(msg.pose.pose.orientation)["yaw"]
        )

    def _scan(self):
        """
        Simulate a scan
        """
        if self._cones is None or self._position is None:
            return

        x, y, t = self._position

        def angle(a, b):
            """
            Angle between two points
            Between -pi and pi
            """
            return (np.arccos(np.dot(
                a / np.linalg.norm(a),
                b / np.linalg.norm(b)
            )) + np.pi) % (2 * np.pi) - np.pi

        # Only consider the cones that the sensor could see
        relevant_cones = [cone
                          for cone in self._cones
                          if math.dist(cone, (x, y)) and self.angle_min < angle(cone, (x, y)) < self.angle_max]

        # Calculate each sensor ray
        ranges = [float(max(
            self.range_min,
            min([*(math.dist(intersection, (x, y))  # Calculate the distance between the intersection and the car
                   for cone in relevant_cones       # For each cone
                   for intersection in circle_line_segment_intersection(
                        cone,
                        CONE_RADIUS,
                        (x, y),
                        (math.cos(theta) * self.range_max + x, math.sin(theta) * self.range_max + y),
                        full_line=False)),          # Get all intersections
                 self.range_max],
                )))
            for theta in np.linspace(t + self.angle_min, t + self.angle_max, LASER_RESOLUTION)]     # All sensor rays

        # Publish lidar
        self.publisher.publish(LaserScan(
            angle_min=float(self.angle_min),
            angle_max=float(self.angle_max),
            angle_increment=float(self.angle_increment),
            time_increment=float(self.time_increment),
            scan_time=float(self.scan_time),
            range_min=float(self.range_min),
            range_max=float(self.range_max),
            ranges=ranges
        ))


def circle_line_segment_intersection(circle_center, circle_radius, pt1, pt2, full_line=True, tangent_tol=1e-9):
    """ Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    REF: https://stackoverflow.com/a/59582674/4441404

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.
                      False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are
                        close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at
                                           which the circle intercepts a line segment.

    Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx ** 2 + dy ** 2)**.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius ** 2 * dr ** 2 - big_d ** 2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2,
             cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2)
            for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct
        # If only considering the segment, filter out intersections that do not fall within the segment
        if not full_line:
            fraction_along_segment = [(xi - p1x) / dx if abs(dx) > abs(dy)
                                      else (yi - p1y) / dy for xi, yi in intersections]
            intersections = [pt for pt, frac in zip(intersections, fraction_along_segment) if 0 <= frac <= 1]
        # If line is tangent to circle, return just one point (as both intersections have same location)
        if len(intersections) == 2 and abs(discriminant) <= tangent_tol:
            return [intersections[0]]
        else:
            return intersections


def main(args=None):
    rclpy.init(args=args)

    lidar = FauxLidar()

    rclpy.spin(lidar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
