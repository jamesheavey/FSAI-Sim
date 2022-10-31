import math
from .map import Map
import vehicle_evaluator.racingline_calculator as rlc
from fs_utils.conversions import quat_to_euler
from .analyse_map import AnalyseMap


class GroundPlanes:

    dirt = "dirt_ground_plane"
    pebble = "pebble_ground_plane"
    brick = "brick_ground_plane"
    asphalt = "asphalt_ground_plane"
    nothing = "ground_plane"


class Lighting:
    """
    Class to store descriptions of the world settings and option names for different environment lighting options
    """
    clear_day = "clear_day"
    dusk = "dusk"
    cloudy_day = "cloudy_day"

    lighting_descriptions = {
        "clear_day": """
    <!-- WORLD SETTINGS -->
    <scene>
      <ambient>1.0 1.0 1.0 1.0</ambient>
      <background>0.7 0.8 1.0 1.0</background>
      <shadows>false</shadows>
    </scene>

    <light name ="sun" type ="directional" >
      <pose>0 0 100 0 -0 0</pose>
      <diffuse>0.8 0.8 0.9 1</diffuse>
      <specular>1 1 1 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>1</cast_shadows>
      <direction>0 1 -0.7</direction>
    </light>
    """,
        "dusk": """
    <!-- WORLD SETTINGS -->
    <scene>
      <ambient>1.0 0.7 0.7 1.0</ambient>
      <background>0.8 0.8 0.8 0.8</background>
      <sky>
      <time>20</time>
      <sunrise>6</sunrise>
      <sunset>20</sunset>
        <clouds>
          <speed>3</speed>
        </clouds>
      </sky>
      <shadows>false</shadows>
    </scene>

    <light name ="sun" type ="directional" >
      <pose>0 -1000 10 0 -0 0</pose>
      <diffuse>0.8 0.7 0.1 1</diffuse>
      <specular>0.8 0.7 0.1 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>1</cast_shadows>
      <direction>0 1 -0.7</direction>
    </light>
    """,
        "cloudy_day": """
    <!-- WORLD SETTINGS -->
    <scene>
      <ambient>1.0 1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8 0.8</background>
      <sky>
      <time>12</time>
      <sunrise>6</sunrise>
      <sunset>20</sunset>
      <clouds>
        <speed>1.0</speed>
        <mean_size>100000000.0</mean_size>
        <humidity>100000000.0</humidity>
      </clouds>
      </sky>
      <shadows>false</shadows>
    </scene>

    <light name ="sun" type ="directional" >
      <pose>0 0 100 0 -0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>1 1 1 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>1</cast_shadows>
      <direction>0 1 -0.7</direction>
    </light>
    """
    }


class GenerateMap:
    """
    Class to create map objects based on centerline. Also converts map object to gazebo world file
    """

    @staticmethod
    def generate_map(name, centerline, track_width=5.0, cone_separation_distance=2.0):
        """
        Returns map object based on centerline and track_width
        """
        # returns yellow, blue, orange cones, start position and angle based on centerline
        yellow_cones, blue_cones, big_cones, start_x, start_y, start_angle = GenerateMap.cone_placement(
            centerline, cone_separation_distance, track_width)
        return Map(
            name=name,
            centerline=centerline,
            blue_cones=blue_cones,
            yellow_cones=yellow_cones,
            orange_cones=big_cones,
            start={
                "position": {"x": start_x, "y": start_y},
                "rotation": {"yaw": start_angle}
            }
        )

    @staticmethod
    def cone_placement(centerline, cone_separation_distance, track_width):
        """
        Generates blue, yellow, and big cone positions based on the centerline
        """
        points = [(p["x"], p["y"]) for p in centerline]
        centerline_spline = rlc.to_spline(map(list, zip(*points)))
        yellow_coordinates = GenerateMap.spline_offset_points(
            centerline_spline, track_width/2.0, cone_separation_distance,  resolution=100)
        blue_coordinates = GenerateMap.spline_offset_points(
            centerline_spline, -track_width/2.0, cone_separation_distance, resolution=100)
        x1, y1 = points[0]
        x2, y2 = points[1]
        # calculate forwards angle from start point to cone locations
        angle_1 = math.atan2((y2 - y1), (x2 - x1)) - math.pi/2.0
        # calculate angle from first cone to second neighbour cone
        angle_2 = angle_1 - math.pi/2.0
        big_cones = [
            {"x": x1 + ((track_width/2.0) * math.cos(angle_1)),
             "y": y1 + ((track_width/2.0) * math.sin(angle_1))},
            {"x": x1 - ((track_width/2.0) * math.cos(angle_1)),
             "y": y1 - ((track_width/2.0) * math.sin(angle_1))},
            {"x": x1 + ((track_width/2.0) * math.cos(angle_1)) + (0.4 * math.cos(angle_2)),
             "y": y1 + ((track_width/2.0) * math.sin(angle_1)) + (0.4 * math.sin(angle_2))},
            {"x": x1 - ((track_width/2.0) * math.cos(angle_1)) + (0.4 * math.cos(angle_2)),
             "y": y1 - ((track_width/2.0) * math.sin(angle_1)) + (0.4 * math.sin(angle_2))}
        ]

        yellow_coordinates = GenerateMap.remove_colliding_cones(
            yellow_coordinates, big_cones)
        blue_coordinates = GenerateMap.remove_colliding_cones(
            blue_coordinates, big_cones)

        return yellow_coordinates, blue_coordinates, big_cones, x1, y1, angle_1 + math.pi/2.0

    @staticmethod
    def spline_offset_points(spline, offset, cone_separation_distance, resolution=100):
        """
        Returns a list of positions of cones generated from the centerline
        """
        offset_points = rlc.generate_offset_points(
            spline, resolution=resolution, distance=offset)
        length = AnalyseMap.calculate_spline_length(offset_points)
        number_of_cones = length/cone_separation_distance
        offset_points = rlc.generate_offset_points(
            spline, resolution=number_of_cones*3.0, distance=offset)
        offset_spline = rlc.to_spline(offset_points)
        offset_points = rlc.generate_offset_points(
            offset_spline, resolution=number_of_cones, distance=0.0)
        offset_list = list(zip(*offset_points))

        return [{"x": x, "y": y} for x, y in offset_list]

    @staticmethod
    def remove_colliding_cones(cones, big_cones, cone_proximity_threshold=1):
        cones = [
            p
            for p in cones
            if all(math.dist((big_cone["x"], big_cone["y"]), (p["x"], p["y"]))
                   > cone_proximity_threshold for big_cone in big_cones)
        ]
        return cones

    @staticmethod
    def from_msg(msg, ground_plane, lighting):
        """
        Converts Map msg into a gazebo world file to be launched
        """
        def pose_to_dict(pose):  # converts pose into x and y coordinates
            return {"x": pose.position.x, "y": pose.position.y}

        return Map(
            name=msg.name,
            centerline=list(map(pose_to_dict, msg.centerline)),
            yellow_cones=list(map(pose_to_dict, msg.yellow_cones)),
            blue_cones=list(map(pose_to_dict, msg.blue_cones)),
            orange_cones=list(map(pose_to_dict, msg.orange_cones)),
            start={"position": {"x": msg.start.position.x, "y": msg.start.position.y},
                   "rotation": {"yaw": quat_to_euler(msg.start.orientation)["yaw"]}},
            loop=msg.loop
        ).generate_map(ground_plane, Lighting.lighting_descriptions[lighting])
