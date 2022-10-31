import math

DOO_PENALTY = 2     # s; Down and out (DOO): hitting cone
OC_PENALTY = 10     # s; Off-course: all four wheels outside the track boundary


class PenaltyTracker:
    def __init__(self, penalty_callbacks=[]):
        self._penalty_callbacks = penalty_callbacks

        self._map_handler = None
        self._cones = []
        self._current_car = None

        # Collect cones, set the ones already hit (one penalty per cone max)
        # Track when car starts to be outside, don't penalise until it's outside again
        self._car_outside = False
        self._current_lap = 0

    def _reset(self):
        self._car_outside = False
        self._current_lap = 0

    def map_update(self, map_handler):
        self._map_handler = map_handler
        self._cones = [cone for group in map_handler.cones for cone in group] if map_handler is not None else []

    def pose_update(self, vehicle_pose):
        if self._map_handler is None or self._current_car is None:
            return

        vehicle_corners = self._get_vehicle_corners(vehicle_pose)

        if not self._car_outside:
            oc = self._is_car_outside(vehicle_corners)
            if oc:
                self._issue_penalty("OC", OC_PENALTY)
            self._car_outside = oc

        if len((collided := self._cones_inside_corners(vehicle_corners, self._cones))) > 0:
            self._issue_penalty("DOO", DOO_PENALTY * len(collided))
            for cone in collided:
                self._cones.remove(cone)

    def _issue_penalty(self, name, time):
        for callback in self._penalty_callbacks:
            callback({"name": name, "penalty_time": time, "lap": self._current_lap})

    def on_start(self, msg):
        self._current_car = msg.car
        self._reset()

    def on_lap(self, lap, time):
        self._current_lap = lap + 1

    def _get_vehicle_corners(self, vehicle_pose):
        forward = (math.cos(vehicle_pose.rotation), math.sin(vehicle_pose.rotation))
        right = (-forward[1], forward[0])

        half_width = self._current_car.width / 2
        half_length = self._current_car.length / 2

        vehicle_corners = (
            (-half_width, -half_length),
            (-half_width, half_length),
            (half_width, half_length),
            (half_width, -half_length)
        )

        position = vehicle_pose.position[:2]

        return [
            [position[i] + corner[0] * forward[i] + corner[1] * right[i] for i in range(2)]
            for corner in vehicle_corners]

    def _is_car_outside(self, vehicle_corners):
        ratio, _, _, dist = min(
            [self._map_handler.get_closest_point_on_centerline(corner) for corner in vehicle_corners],
            key=lambda c: c[3])
        map_width = self._map_handler.get_map_width_at_ratio(ratio)

        return dist > map_width

    def _cones_inside_corners(self, vehicle_corners, cones):
        return [
            cone
            for cone in cones
            if (self._is_inside_triangle(vehicle_corners[0], vehicle_corners[1], vehicle_corners[2], cone) or
                self._is_inside_triangle(vehicle_corners[2], vehicle_corners[3], vehicle_corners[0], cone))
        ]

    def _is_inside_triangle(self, p1, p2, p3, p):
        """
        Check if point (p) is inside a triangle formed by p1-3
        REF: https://www.geeksforgeeks.org/check-whether-a-given-point-lies-inside-a-triangle-or-not/
        """
        def area(x1, y1, x2, y2, x3, y3):
            return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                        + x3 * (y1 - y2)) / 2.0)

        # Calculate area of triangle ABC
        A = area(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1])

        # Calculate area of triangle PBC
        A1 = area(p[0], p[1], p2[0], p2[1], p3[0], p3[1])

        # Calculate area of triangle PAC
        A2 = area(p1[0], p1[1], p[0], p[1], p3[0], p3[1])

        # Calculate area of triangle PAB
        A3 = area(p1[0], p1[1], p2[0], p2[1], p[0], p[1])

        # Check if sum of A1, A2 and A3
        # is same as A
        if(A == A1 + A2 + A3):
            return True
        else:
            return False
