import math
import numpy as np
from .generate_map import GenerateMap


class SemiStaticMaps:
    """
    Class to generate semi static maps from a generated centerline and some parameters
    """

    @staticmethod
    def get_map(name, track_width, radius, length, height, cone_separation_distance):
        """
        Returns the requested semi static map object
        """
        if name == "small_track_custom":
            return SemiStaticMaps.small_track(track_width, cone_separation_distance, length, radius)
        if name == "rectangle_track":
            return SemiStaticMaps.rect_map(track_width, cone_separation_distance, length, height, radius)
        if name == "circle_track":
            return SemiStaticMaps.circle_map(radius, track_width, cone_separation_distance)
        else:
            return None

    @staticmethod
    def circle_map(radius=17.5, track_width=5.0, cone_separation_distance=2.0):
        """
        Generates circle map based on radius parameter (default = 17.5)
        """
        centerline = []
        for theta in np.linspace(math.pi * 2.0, 0.0, 50):
            centerline.append({
                "x": math.cos(theta) * radius,
                "y": math.sin(theta) * radius,
            })
        map = GenerateMap.generate_map("circle_map", centerline, track_width, cone_separation_distance)
        return map

    @staticmethod
    def rect_map(track_width=5.0, cone_separation_distance=2.0, length=60.0, height=40.0, corner_radius=10.0):
        """
        Generates rectangle map based on length, height, and corner radius parameters.
        Defaults (length=60.0, height=40.0, corner_radius=10.0) produce random values
        """
        centerline = []
        cone_separation_distance = 2.0
        corner_length = (math.pi * corner_radius) / 2.0
        # -- Add centerline --
        # First straight - bottom left
        for x in np.linspace(0.0, -length/2.0,
                             int(length/(2 * cone_separation_distance)), endpoint=False):
            centerline.append({
                "x": x,
                "y": 0.0,
            })
        # Bottom left corner centerline
        for theta in np.linspace(3.0*math.pi/2.0, math.pi, int(2.0 * corner_length/cone_separation_distance)):
            centerline.append({
                "x": -length/2.0 + (math.cos(theta) * corner_radius),
                "y": corner_radius + (math.sin(theta) * corner_radius),
            })
        # Left height centerline
        for y in np.linspace(corner_radius, height + corner_radius,
                             int(height/cone_separation_distance), endpoint=False)[1:]:
            centerline.append({
                "x": -length/2.0 - corner_radius,
                "y": y,
            })
        # Top left corner centerline
        for theta in np.linspace(math.pi, math.pi/2.0, int(2.0 * corner_length/cone_separation_distance)):
            centerline.append({
                "x": -length/2.0 + (math.cos(theta) * corner_radius),
                "y": height + corner_radius + (math.sin(theta) * corner_radius),
            })
        # Top length centerline
        for x in np.linspace(-length/2.0, length/2.0,
                             int(length/(2 * cone_separation_distance)), endpoint=False)[1:]:
            centerline.append({
                "x": x,
                "y": height + (2 * corner_radius),
            })
        # Top right corner centerline
        for theta in np.linspace(math.pi/2.0, 0.0, int(2.0 * corner_length/cone_separation_distance)):
            centerline.append({
                "x": length/2.0 + (math.cos(theta) * corner_radius),
                "y": height + corner_radius + (math.sin(theta) * corner_radius),
            })
        # Right height centerline
        for y in np.linspace(height + corner_radius, corner_radius,
                             int(height/cone_separation_distance), endpoint=False)[1:]:
            centerline.append({
                "x": length/2.0 + corner_radius,
                "y": y,
            })
        # Bottom right corner centerline
        for theta in np.linspace(2.0 * math.pi, 3.0*math.pi/2.0, int(2.0 * corner_length/cone_separation_distance)):
            centerline.append({
                "x": length/2.0 + (math.cos(theta) * corner_radius),
                "y": corner_radius + (math.sin(theta) * corner_radius),
            })
        # Last straight - bottom right centerline
        for x in np.linspace(length/2.0, 0.0,
                             int(length/(2 * cone_separation_distance)), endpoint=False)[1:]:
            centerline.append({
                "x": x,
                "y": 0.0,
            })
        map = GenerateMap.generate_map("rect_map", centerline, track_width, cone_separation_distance)
        return map

    @staticmethod
    def small_track(track_width=5.0,
                    cone_separation_distance=2.0,
                    straight_length=20.0,
                    curve_radius=15.0):
        """
        Manually created map generated from 2 parameters, straight length, and corner radius
        """
        centerline = []
        curve_length = (math.pi * curve_radius) / 2.0
        # Straight 1
        for y in np.linspace(0.0, straight_length, int(straight_length), endpoint=False):
            centerline.append({
                "x": 0.0,
                "y": y,
            })
        # Corner 1
        for theta in np.linspace(0.0, math.pi/2.0, int(curve_length), endpoint=False):
            centerline.append({
                "x": (math.cos(theta) * curve_radius) - curve_radius,
                "y": straight_length + (math.sin(theta) * curve_radius),
            })
        # Corner 2
        for theta in np.linspace(3.0 * math.pi/2.0, math.pi/2.0, int(curve_length), endpoint=False):
            centerline.append({
                "x": (math.cos(theta) * (curve_radius/2.0)) - curve_radius,
                "y": straight_length + (1.5 * curve_radius) + (math.sin(theta) * (curve_radius/2.0)),
            })
        # Straight 2
        for x in np.linspace(-curve_radius, curve_radius, int(2.0 * curve_radius), endpoint=False):
            centerline.append({
                "x": x,
                "y": straight_length + (2.0 * curve_radius),
            })
        # Corner 3
        for theta in np.linspace(math.pi/2.0, 0.0, int(curve_length), endpoint=False):
            centerline.append({
                "x": (math.cos(theta) * curve_radius) + curve_radius,
                "y": straight_length + curve_radius + (math.sin(theta) * curve_radius),
            })
        # Straight 3
        for y in np.linspace(
                straight_length + curve_radius, 0.0, int(straight_length + curve_radius), endpoint=False):
            centerline.append({
                "x": 2.0 * curve_radius,
                "y": y,
            })
        # Final corner
        for theta in np.linspace(2.0 * math.pi, math.pi, int(curve_length * 2.0), endpoint=False):
            centerline.append({
                "x": (math.cos(theta) * curve_radius) + curve_radius,
                "y": (math.sin(theta) * curve_radius),
            })

        map = GenerateMap.generate_map("small_track", centerline, track_width, cone_separation_distance)
        return map
