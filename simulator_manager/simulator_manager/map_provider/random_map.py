import random
import numpy as np
from scipy.spatial import ConvexHull
import math
from .generate_map import GenerateMap
from .modify_map import ModifyMap
from .fsai_rules import FSAIRules
from .analyse_map import AnalyseMap


class MapSize:
    """
    Class to define map size options
    """
    small = 30
    medium = 50
    large = 100


class MapComplexity:
    """
    Class to define map complexity options
    """
    basic = (0.8, 4)
    medium = (0.5, 3)
    complex = (0.2, 2)


class RandomMap:
    """
    Class to generate random map based on user parameters
    """
    seed_index = 0
    point_count = 20  # magic number, doesn't need to be customized

    @staticmethod
    def get_map(map_name, track_width, map_seed,
                map_size=MapSize.medium, map_complexity=MapComplexity.medium,
                allow_intersections=False, cone_separation_distance=2):
        """
        Creates the randomly generated map, and calls relevant functions for track manipulation based on user parameters
        """
        random.seed(map_seed)  # Set seed for subsequent random number generation
        proper_map = False

        while not proper_map:
            print(f"Creating map with seed = {map_seed}")
            number_of_points = 0
            # Generate random points but ensure they are above minimum number for map size
            while number_of_points < (map_size/5.0):
                random_points = RandomMap.generate_points(RandomMap.point_count, map_size)
                random_points = ModifyMap.apply_mask(random_points, map_size)
                number_of_points = len(random_points)
            # Create convex hull of random points
            points = RandomMap.convex_hull(random_points)
            points = RandomMap.generate_center_points(points, map_size, track_width, allow_intersections)
            if not FSAIRules.adhere_to_rules(points, track_width, allow_intersections):
                continue
            # TODO make the track modifications based on the variance of radius
            for i in range(random.randrange(int(1/map_complexity[0]) - 1, int(2/map_complexity[0]) + 1)):
                # Modify track
                points = RandomMap.add_n_corners(points, map_size, map_complexity, track_width, allow_intersections)

            # Check if modified track adheres to FS-AI rules
            if not FSAIRules.adhere_to_rules(points, track_width, allow_intersections):
                print("Map Generation Failed")
                continue

            if random.random() > 0.5:  # Randomly reverse track direction
                points.reverse()
            # Create spline from track
            spline, _ = AnalyseMap.to_spline(map(list, zip(*points)))
            # Create centerline from spline
            centerline = GenerateMap.spline_offset_points(spline, 0.0, 2.0,  resolution=100)
            proper_map = True
            print("Map Generated")
        # Return Generated map
        return GenerateMap.generate_map("generated", centerline, track_width, cone_separation_distance)

    @staticmethod
    def add_n_corners(points, map_size, map_complexity, track_width, allow_intersections):
        """
        Modifies the track to create more interesting tracks.
        It does this by adding corners
        """

        for j in range(int(2/map_complexity[0])):
            points = ModifyMap.add_corners(points, map_complexity, track_width, allow_intersections, map_size)

        return points

    @staticmethod
    def generate_points(point_count, map_size):
        """
        Generates a set of random coordinates
        """
        points = []
        min_distance = 5
        # Generate a number of random coordinates within the map size contraint
        for i in range(point_count):
            test_point = (random.randrange(-map_size/2.0, map_size/2.0), random.randrange(-map_size/2.0, map_size/2.0))
            distances = [math.dist(test_point, p) for p in points]
            too_close = [x for x in distances if x < min_distance]
            if len(too_close) == 0:
                points.append(test_point)
        return points

    @staticmethod
    def convex_hull(points):
        """
        Creates the smallest convex polygon that contains all points and returns the points that make up the vertices
        """
        hull = ConvexHull(points)
        # vertices is the coordinates that make up the corners of the convex polygon
        vertices = [(points[vertex][0], points[vertex][1]) for vertex in hull.vertices]
        return vertices

    @staticmethod
    def generate_center_points(points, map_size, track_width, allow_intersections):
        """
        Generates a random number of points a center the center region based on map size
        """
        number_of_center_points = 0
        # TODO adjust the center of the map based on the mask applied
        break_count = 0
        while number_of_center_points < 2 and break_count < 10:
            test_points = list(points)
            point_index = random.randrange(len(points))
            y_multiplier = np.sign(points[point_index][1])
            x_multiplier = np.sign(points[point_index][0])
            center_point = (x_multiplier * random.randrange(int(map_size/6.0), int(map_size/3.0)),
                            y_multiplier * random.randrange(int(map_size/6.0), int(map_size/3.0)))
            test_points[point_index] = center_point
            too_close = AnalyseMap.too_close_to_other_points(test_points, point_index, track_width)
            if (too_close or not FSAIRules.adhere_to_rules(test_points, track_width, allow_intersections)):
                break_count += 1
                continue
            number_of_center_points += 1
            points[point_index] = (x_multiplier * random.randrange(int(map_size/6.0), int(map_size/3.0)),
                                   y_multiplier * random.randrange(int(map_size/6.0), int(map_size/3.0)))
        return points

    @staticmethod
    def fix_track(points, allow_intersections, track_width):
        """
        This function aims to remove all corners that have too tight of a radius and remove intersections
        """
        sharp_corners = True
        loop_count = 3
        # continue to smooth corners until there are no corners with radius lower than minimum radius value
        while sharp_corners and loop_count <= 0:
            radii, x_new, y_new, _ = AnalyseMap.calculate_track_radii(points)
            tight_corners = AnalyseMap.find_tight_corners(radii, x_new, y_new, track_width, 5)
            tight_corner_points = AnalyseMap.tight_corner_points(tight_corners, points)

            if len(tight_corner_points) > 0:
                points = FSAIRules.remove_tight_corners(points, track_width)
                loop_count -= 1
            else:
                sharp_corners = False
        if not allow_intersections:
            for i in range(3):
                points, intersections = FSAIRules.remove_intersections(points)
                if len(intersections) == 0:
                    break
        else:
            print("Intersections allowed")
        return points
