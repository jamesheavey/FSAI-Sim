import math
import numpy as np
from scipy import interpolate
import vehicle_evaluator.racingline_calculator as rlc


class AnalyseMap:

    @staticmethod
    def find_tight_corners(radii, x_new, y_new, track_width, minimum_radius):
        """
        This function returns spline point coordinates of corners with a radii that are below 2.5m
        """
        tight_corners = []
        for i in range(len(radii)):
            if radii[i] < (minimum_radius - track_width/2.0):
                tight_corners.append([x_new[i], y_new[i]])
        return tight_corners

    @staticmethod
    def determine_intersections(points):
        """
        Function to find intersections at any point on the created track
        """
        spline, _ = AnalyseMap.to_spline(map(list, zip(*points)))
        spline_points = rlc.generate_offset_points(spline, 50, 0)
        track_length = AnalyseMap.calculate_spline_length(spline_points)
        spline_points = rlc.generate_offset_points(spline, track_length/2, 0)
        points = list(zip(*spline_points))
        intersection = (0, 0)
        intersections = []
        positions = []
        i = 0
        j = 0
        for i in range(len(points)-1):
            for j in range(len(points)-1):
                intersection = AnalyseMap.find_intersection(points[i], points[i+1], points[j], points[j+1])
                if intersection is None or points[i+1] == points[j] or points[j+1] == points[i]:
                    continue
                vertices = [points[i], points[j], points[i+1], points[j+1]]
                if not AnalyseMap.is_inside_polygon(vertices, intersection):
                    continue
                positions.append((i, j))
                intersections.append(intersection)
        return intersections, positions, points

    @staticmethod
    def find_overlaps(points, track_width):
        overlaps = [(i, j)
                    for j in range(len(points)-1)
                    for i in range(len(points)-1)
                    if not j - 2 < i < j + 2
                    and math.dist(points[i], points[j]) < track_width]
        return overlaps

    @staticmethod
    def find_intersection(point_a1, point_a2, point_b1, point_b2):
        """
        Function to find the point of intersection between lines made through 2 pairs of points
        """
        # Implementation of REF: https://rosettacode.org/wiki/Find_the_intersection_of_two_lines#Python
        Ax1, Ay1 = point_a1
        Ax2, Ay2 = point_a2

        Bx1, By1 = point_b1
        Bx2, By2 = point_b2

        d = (By1 - By2) * (Ax1 - Ax2) - (Bx1 - Bx2) * (Ay1 - Ay2)
        if d:
            uA = ((Bx2 - Bx1) * (Ay1 - By1) - (By2 - By1) * (Ax1 - Bx1)) / d
            uB = ((Ax2 - Ax1) * (Ay1 - By1) - (Ay2 - Ay1) * (Ax1 - Bx1)) / d
        else:
            return
        if not(0 <= uA <= 1 and 0 <= uB <= 1):
            return
        x = Ax1 + uA * (Ax2 - Ax1)
        y = Ay1 + uA * (Ay2 - Ay1)

        return (x, y)

    @staticmethod
    def find_angle(point_1, point_2):
        """
        Returns the angle between 2 points
        """
        return math.atan2((point_2[1] - point_1[1]), (point_2[0] - point_1[0]))

    @staticmethod
    def find_midpoint(point_1, point_2):
        """
        returns the distance, midpoint, and angle between 2 points
        """
        distance = math.dist(point_1, point_2)/2.0
        angle = AnalyseMap.find_angle(point_1, point_2)
        midpoint = (point_1[0] + (distance * math.cos(angle)), point_1[1] + (distance * math.sin(angle)))
        return distance, midpoint, angle

    @staticmethod
    def distances_to_other_points(points, index):
        """
        return the distances between a point and every other point in the list
        """
        distances = []
        test_point = points[index]
        distances = [math.dist(test_point, p) for p in points]
        return distances

    @staticmethod
    def too_close_to_other_points(points, index, min_distance=5):
        """
        Returns a bool for if any point is too close to a specified point
        """
        distances = AnalyseMap.distances_to_other_points(points, index)
        too_close = [x for x in distances if x < min_distance]
        return len(too_close) > 1

    @staticmethod
    def inside_map_size(point, map_size, track_width):
        return (abs(point[0]) < ((map_size/2.0) - track_width) and abs(point[1]) < ((map_size/2.0) - track_width))

    @staticmethod
    def calculate_spline_length(points):
        """
        Calculates spline length, used for calculating number of cones
        """
        dist_array = []
        x, y = points
        for i in range(0, len(x)-1):
            dist_array.append((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)
        length = sum(np.sqrt(dist_array))
        return length

    @staticmethod
    def calculate_track_radii(points):
        """
        This derives a track radius at each point along a spline
        It returns the radius values, the corresponding coordinates, and the input array range
        """
        # REF: https://stackoverflow.com/questions/45104319/how-to-evaluate-spline-derivatives-from-splprep
        spline, u = AnalyseMap.to_spline(map(list, zip(*points)))
        u_new = np.linspace(u.min(), u.max(), 200)
        x_new, y_new = interpolate.splev(u_new, spline, der=0)

        xd, yd = interpolate.splev(u_new, spline, der=1)
        xdd, ydd = interpolate.splev(u_new, spline, der=2)
        # REF: https://www.math24.net/curvature-radius
        curvature = np.abs(xd * ydd - yd * xdd) / np.power(xd ** 2 + yd ** 2, 3 / 2)

        radii = 1.0 / curvature

        return radii, x_new, y_new, u_new

    @staticmethod
    def tight_corner_points(tight_corners, points):
        """
        This function finds points within a distance from any tight corners and returns these points
        """
        point_proximity_threshold = 2
        corner_points = [
            i
            for point in tight_corners for i, p in enumerate(points)
            if math.dist(point, points[i]) < point_proximity_threshold
        ]
        return corner_points

    @staticmethod
    def to_spline(points):
        """
        Given a set of numpy points, calculate spline
        """
        x, y = points
        try:
            x_wrap = np.r_[x, x[0]]
            y_wrap = np.r_[y, y[0]]
            tck, u = interpolate.splprep([x_wrap, y_wrap], s=0, per=True)
        except Exception:
            tck, u = interpolate.splprep([x, y], s=0, per=True)
        return tck, u

    @staticmethod
    def on_segment(p, q, r):
        """
        Determines if a point is on a segment
        """
        # REF: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
        print(f"on segment = {(q[0] <= max(p[0], r[0]))}")
        return ((q[0] <= max(p[0], r[0])) and
                (q[0] >= min(p[0], r[0])) and
                (q[1] <= max(p[1], r[1])) and
                (q[1] >= min(p[1], r[1])))

    @staticmethod
    def orientation(p, q, r):
        """
        To find orientation of ordered triplet (p, q, r).
        The function returns following values
        0 --> p, q and r are colinear
        1 --> Clockwise
        2 --> Counterclockwise
         """
        # REF: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
        val = (((q[1] - p[1]) *
                (r[0] - q[0])) -
               ((q[0] - p[0]) *
                (r[1] - q[1])))

        if val == 0:
            return 0
        if val > 0:
            return 1  # Collinear
        else:
            return 2  # Clock or counterclock

    @staticmethod
    def do_intersect(p1, q1, p2, q2):
        """
        Find the four orientations needed for general case
        """
        # REF: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/

        o1 = AnalyseMap.orientation(p1, q1, p2)
        o2 = AnalyseMap.orientation(p1, q1, q2)
        o3 = AnalyseMap.orientation(p2, q2, p1)
        o4 = AnalyseMap.orientation(p2, q2, q1)

        # General case
        return (o1 != o2) and (o3 != o4)

    @staticmethod
    def is_inside_polygon(points, p):
        """
        Returns true if the point p lies
        inside the polygon[] with n vertices
        """
        # REF: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
        n = len(points)

        # There must be at least 3 vertices
        # in polygon
        if n < 3:
            return False

        # Create a point for line segment
        # from p to infinite
        extreme = (10000, p[1])
        count = i = 0

        while True:
            next = (i + 1) % n

            # Check if the line segment from 'p' to
            # 'extreme' intersects with the line
            # segment from 'polygon[i]' to 'polygon[next]'
            if (AnalyseMap.do_intersect(points[i],
                                        points[next],
                                        p, extreme)):

                # If the point 'p' is colinear with line
                # segment 'i-next', then check if it lies
                # on segment. If it lies, return true, otherwise false
                if AnalyseMap.orientation(points[i], p,
                                          points[next]) == 0:
                    return AnalyseMap.on_segment(points[i], p,
                                                 points[next])

                count += 1

            i = next

            if (i == 0):
                return (count % 2 == 1)  # Return true if count is odd, false otherwise
