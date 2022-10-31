import os
import json
from fs_utils.conversions import dict_to_pose
from fs_msgs.msg import Map as MapMsg
from .cone_generator import ConeGenerator, WORLD_TEMPLATE
from ament_index_python.packages import get_package_share_directory


class Map:
    path = os.path.join(get_package_share_directory("fs_description"), "maps", "worlds/generated.world")

    def __init__(self, name,
                 centerline=[],
                 yellow_cones=[], blue_cones=[], orange_cones=[],
                 start={}, loop=True):
        self.name = name
        self.centerline = centerline
        self.yellow_cones = yellow_cones
        self.blue_cones = blue_cones
        self.orange_cones = orange_cones
        self.start = start
        self.loop = loop

    def to_msg(self):  # creates a message of map properties
        def to_pose(d): return dict_to_pose({"position": d})  # returns input as pose
        return MapMsg(
            centerline=list(map(to_pose, self.centerline)),
            yellow_cones=list(map(to_pose, self.yellow_cones)),
            blue_cones=list(map(to_pose, self.blue_cones)),
            orange_cones=list(map(to_pose, self.orange_cones)),
            start=dict_to_pose(self.start),
            name=self.name,
            loop=self.loop
        )

    def generate_map(self, ground_plane, cloudy_day):
        cones = "\n".join(  # combines all cones into 1 list in the Gazebo world format
            [ConeGenerator.get_blue_cone(id=i, **coord)
             for i, coord in enumerate(self.blue_cones)] +
            [ConeGenerator.get_yellow_cone(id=i, **coord)
             for i, coord in enumerate(self.yellow_cones)] +
            [ConeGenerator.get_big_cone(id=i, **coord)
             for i, coord in enumerate(self.orange_cones)]
        )
        # input cones into wold template
        world = WORLD_TEMPLATE.format(lighting=cloudy_day, ground_plane=ground_plane, models=cones)

        with open(self.path, "w") as f:
            f.write(world)  # saves world file

        return self

    @staticmethod
    def from_umdf(path):
        with open(path) as f:
            umdf = json.load(f)
        return Map(
            umdf["name"],
            umdf["centreline"],
            umdf["yellow_cones"],
            umdf["blue_cones"],
            umdf["orange_cones"],
            umdf["start"],
            umdf.get("loop", True)
        )
