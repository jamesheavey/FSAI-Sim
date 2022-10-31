import math
import numpy as np
import random
from .analyse_map import AnalyseMap
from .fsai_rules import FSAIRules
from fs_utils.conversions import dict_to_pose, pose_to_dict


class ConeMissing:
    """
    Class to define cone missing
    Number sets the average percentage of cones that are missing.
    Percentage = value * 100
    """
    none = 0
    low = 0.01
    medium = 0.02
    high = 0.03


class ConeMisplacement:
    """
    Class to define cone misplacement
    Number sets the distance in which 95% of the misplacement is set according to a normal distribution
    Distance = value * 2
    """
    none = 0
    low = 0.05
    medium = 0.1
    high = 0.15


class ModifyMap:

    masks = [np.array([[1, 1, 1],
                       [1, 1, 1],
                       [0, 0, 0]]),

             np.array([[0, 0, 0],
                       [1, 1, 1],
                       [1, 1, 1]]),

             np.array([[1, 1, 0],
                       [1, 1, 0],
                       [1, 1, 0]]),

             np.array([[0, 1, 1],
                       [0, 1, 1],
                       [0, 1, 1]]),

             np.array([[1, 1, 1],
                       [0, 0, 1],
                       [1, 1, 1]]),

             np.array([[1, 1, 1],
                       [1, 0, 0],
                       [1, 1, 1]]),

             np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 0, 1]]),

             np.array([[1, 0, 1],
                       [1, 0, 1],
                       [1, 1, 1]])
             ]

    @staticmethod
    def apply_mask(points, map_size):
        """
        This function applies a random binary mask from the mask arrays to the points that form the track.
        """
        # pick random mask from array in MaskArrays class
        mask = ModifyMap.masks[random.randrange(0, len(ModifyMap.masks))]
        n = map_size
        new_points = []

        # scale mask to map size
        new_mask = np.kron(mask, np.ones((n, n)))

        # multiply either every points with the mask value
        for i, point in enumerate(points):
            masked_x = points[i][0] * new_mask[int(point[0])][int(point[1])]
            masked_y = points[i][1] * new_mask[int(point[0])][int(point[1])]
            new_points.append((masked_x, masked_y))

        # create new array of masked points
        masked_points = [i for i in new_points if i != (0, 0)]

        return masked_points

    @staticmethod
    def push_apart(points, distance=7):
        """
        Pushes apart points that are within a minimum distance from another point
        """
        # REF: https://www.gamasutra.com/blogs/GustavoMaciel/20131229/207833/Generating_Procedural_Racetracks.php
        dist = distance
        i = 0
        j = 0
        too_close = [1]
        while len(too_close) > 0:
            distances = []
            for i in range(len(points)):
                for j in range(len(points)):
                    if j == i or math.dist(points[i], points[j]) > dist:
                        continue
                    d = (points[i][0] - points[j][0], points[i][1] - points[j][1])
                    hl = math.dist(points[i], points[j])
                    # Check to prevent division by ~0 error
                    d = (d[0] / hl, d[1] / hl) if abs(hl) > 0.01 else (0.01, 0.01)
                    dif = dist - hl
                    d = (d[0] * dif, d[1] * dif)
                    points[j] = (points[j][0] + d[0], points[j][1] + d[1])
                    points[i] = (points[i][0] - d[0], points[i][1] - d[1])
                    distances.append(math.dist(points[i], points[j]))
            too_close = [x for x in distances if x < dist]
        return points

    @staticmethod
    def add_corners(points, complexity, track_width, allow_intersections, map_size):
        """
        returns a modified list of vertices including added corners for some longer portions of the track
        """
        i = 1
        skip_value, offset_adjust = complexity
        j = len(points)
        test_points = []
        while i < j - 1:
            distance, midpoint, angle = AnalyseMap.find_midpoint(points[i], points[i+1])
            test_points = list(points)
            # only add an offset if points are more than 5 meters apart
            if distance < 5.0 or random.random() < skip_value or distance/offset_adjust < 1.0:
                i += 1
                continue
            else:
                # checks if int(distance/offset_adjust) would be 0, as this creates a 0 range in randrange
                # offset distance is relative to distance between 2 points
                offset_distance = random.randrange(int(- distance/offset_adjust), int(distance/offset_adjust))
                offset_angle = random.randrange(int((angle*180 / math.pi) - 25), int((angle*180 / math.pi) + 25))
                midpoint_offset = (
                    midpoint[0] + (offset_distance * math.sin(offset_angle * (math.pi / 180))),
                    midpoint[1] + (offset_distance * math.cos(offset_angle * (math.pi / 180)))
                )
                if not AnalyseMap.inside_map_size(midpoint_offset, map_size, track_width):
                    i += 1
                    continue
                test_points.insert(i+1, midpoint_offset)
                too_close = AnalyseMap.too_close_to_other_points(test_points, i+1, track_width)
                # If test point is too close to any others or new spline doesn't adhere to rules, don't add it
                if (too_close or not FSAIRules.adhere_to_rules(test_points, track_width, allow_intersections)):
                    i += 1
                    continue
                # Add test point to points
                else:
                    points.insert(i+1, midpoint_offset)  # insert offset coordinates to points
                    j += 1
                    i += 2
        return points

    @staticmethod
    def rotate_points(center, points, theta):
        """
        Function to return a rotated set of points
        Points are rotated if they are within a distance
        """
        rotated_points = []
        ox, oy = center
        for px, py in points:
            if math.dist(center, (px, py)) < 20:
                angle = theta * (math.pi/180)
                qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
                qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
                rotated_points.append((qx, qy))
            else:
                rotated_points.append((px, py))
        return rotated_points

    @staticmethod
    def get_fuzzed_map(seed, msg, cone_misplacement, cone_missing):
        random.seed(seed)

        yellow_coordinates = list(map(pose_to_dict, msg.yellow_cones))
        blue_coordinates = list(map(pose_to_dict, msg.blue_cones))

        if cone_misplacement != 0:  # If a cone misplacement is set, call functions for modifying cones
            yellow_coordinates = ModifyMap.cone_misplacement(yellow_coordinates, cone_misplacement)
            blue_coordinates = ModifyMap.cone_misplacement(blue_coordinates, cone_misplacement)
        if cone_missing != 0:  # If a cone missing is set, call functions for modifying cones
            yellow_coordinates = ModifyMap.cone_missing(yellow_coordinates, cone_missing)
            blue_coordinates = ModifyMap.cone_missing(blue_coordinates, cone_missing)

        msg.yellow_cones = list(map(dict_to_pose, yellow_coordinates))
        msg.blue_cones = list(map(dict_to_pose, blue_coordinates))

        return msg

    @staticmethod
    def cone_misplacement(cones, cone_misplacement):
        """
        Function to randomly misplace all cones by a small amount
        """
        # Normal distribution used. As configured, 95% of values lie within +/- (2 * cone_misplacement value)
        # Therefore for low, 95% of values lie within 0.1
        # For medium, 0.2, and for high, 0.3
        cones = [{**p,
                  "position": {**p["position"],
                               "x": p["position"]["x"] + (random.normalvariate(0, cone_misplacement)),
                               "y": p["position"]["y"] + (random.normalvariate(0, cone_misplacement))
                               }}
                 for p in cones
                 ]
        return cones

    @staticmethod
    def cone_missing(cones, cone_missing):
        """
        Function to randomly remove some random cones
        """
        cone_missing = min(max(cone_missing, 0), 1)
        cones = [
            p
            for p in cones
            if random.uniform(0, 1) < (1 - cone_missing)
        ]
        return cones
