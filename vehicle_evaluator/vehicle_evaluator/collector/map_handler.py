import numpy as np
from scipy import interpolate
from fs_utils.conversions import pose_to_dict, dict_to_pose
from ..racingline_calculator import calculate_racing_line, to_spline
from fs_msgs.msg import Map
from fs_utils.conversions import map_to_dict


class MapHandler:
    """
    Hold all the map information
    """

    def __init__(self, map_msg):
        if not map_msg.loop:
            raise Exception("non looping maps are not supported")
        self._map = map_msg
        # Get centrepoints, fit a spline
        self._centre_points = list(map(pose_to_dict, map_msg.centerline))
        self._centre_arr = tuple(map(
            np.array,
            zip(*[(p.position.x, p.position.y) for p in map_msg.centerline])
        ))
        self._centre_spline = to_spline(self._centre_arr)

        # Find start, calculate the width of the track based on the distance of the orange cones
        self._start = pose_to_dict(map_msg.start)
        self._width_at_start = np.sqrt(min(
            [(map_msg.start.position.x - p.position.x) ** 2 + (map_msg.start.position.y - p.position.y) ** 2
             for p in map_msg.orange_cones]
        ))

        # Calculate racingline length
        self._track_length = self._calculate_spline_length(self._centre_spline)

        # Calculate racingline and fit a spline
        _, _, self._racingline_points = calculate_racing_line(
            self._centre_spline, self._width_at_start * 0.8, res=self._track_length // 5, alpha=0.5)
        self._racingline_spline = to_spline(list(zip(*self._racingline_points)))

    def _calculate_spline_length(self, spline, resolution=1000):
        """
        Calculate the length of a spline by taking resolution number of samples
        """
        points = list(zip(*interpolate.splev(np.linspace(0, 1, int(resolution + 1)), spline)))
        return sum(np.linalg.norm(np.array(p1) - np.array(p2)) for p1, p2 in zip(points[1:], points[:-1]))

    @property
    def map_msg(self, resolution=200):
        """
        Return calculated values as a map message
        Use centerline for calculated centerline (higher resolution than the actual centerline)
        Use yellow_cones for racingline
        """
        ctr = list(zip(*interpolate.splev(np.linspace(0, 1, resolution + 1), self._centre_spline)))
        rcl = list(zip(*interpolate.splev(np.linspace(0, 1, resolution + 1), self._racingline_spline)))

        def to_points(pts):
            return list(map(lambda p: dict_to_pose({"position": {"x": p[0], "y": p[1]}}), pts))

        return Map(
            centerline=to_points(ctr),
            yellow_cones=to_points(rcl),
        )

    def _get_spline_angle(self, spline, point):
        """
        Calculate the tangent angle of a spline at some point [0, 1]
        """
        e = 1e-4
        x1, y1 = interpolate.splev(point, spline)
        x2, y2 = interpolate.splev(point + e, spline)
        return np.arctan2(y2 - y1, x2-x1)

    def _get_closest_point_on_spline(self, pos, spline, resolution=5000):
        """
        Find closest point on a spline by taking a bunch of samples, and selecting the closest
        Returns x, y and the proportion of spline the point is on
        """
        x, y = pos
        i, (mx, my) = min(
            enumerate(zip(*interpolate.splev(np.linspace(0, 1, resolution + 1), spline))),
            key=lambda v: (v[1][0] - x) ** 2 + (v[1][1] - y) ** 2
        )
        return i / resolution, mx, my

    def get_closest_point_on_racingline(self, pos, resolution=5000):
        """
        Find the closest point on the racingline to a given pos
        Also calculate the tangent angle of the racing line at that point
        """
        i, x, y = self._get_closest_point_on_spline(pos, self._racingline_spline, resolution)
        racingline_angle = self._get_spline_angle(self._racingline_spline, i)
        distance = np.sqrt((x - pos[0]) ** 2 + (y - pos[1]) ** 2)
        return i, x, y, racingline_angle, distance

    def get_closest_point_on_centerline(self, pos, resolution=5000):
        """
        Find the closest point on the centerline to a given pos
        """
        i, x, y = self._get_closest_point_on_spline(pos, self._centre_spline, resolution)
        distance = np.sqrt((x - pos[0]) ** 2 + (y - pos[1]) ** 2)
        return i, x, y, distance

    def _spline_to_dict(self, spline, resolution=100):
        return [{"position": {"x": x, "y": y}}
                for x, y in zip(*interpolate.splev(np.linspace(0, 1, resolution + 1), spline))]

    def to_dict(self):
        return {
            **map_to_dict(self._map),
            "racingline": self._spline_to_dict(self._racingline_spline)
        }

    def get_map_width_at_ratio(self, i):
        x, y = interpolate.splev(i, self._centre_spline)

        def find_closest_cone(cones):
            return min(cones,
                       key=lambda pose: (pose.position.x - x) ** 2 + (pose.position.y - y) ** 2)

        yellow_cone = find_closest_cone(self._map.yellow_cones)
        y_x, y_y = yellow_cone.position.x, yellow_cone.position.y
        blue_cone = find_closest_cone(self._map.blue_cones)
        b_x, b_y = blue_cone.position.x, blue_cone.position.y

        _, _, _, y_dist = self.get_closest_point_on_centerline((y_x, y_y))
        _, _, _, b_dist = self.get_closest_point_on_centerline((b_x, b_y))

        return (y_dist + b_dist) / 2

    @property
    def cones(self):
        def cones_to_tuples(cones):
            return [(cone.position.x, cone.position.y) for cone in cones]
        return (cones_to_tuples(self._map.yellow_cones),
                cones_to_tuples(self._map.blue_cones),
                cones_to_tuples(self._map.orange_cones))
