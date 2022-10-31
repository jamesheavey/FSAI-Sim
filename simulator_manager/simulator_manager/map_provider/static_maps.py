from .map import Map
from ament_index_python.packages import get_package_share_directory
import os


class StaticMaps:
    """
    Class to generate static maps from file and generate centerline for semi static maps
    """

    static_maps = {
        "small_track":
        Map.from_umdf(
            os.path.join(
                get_package_share_directory("fs_description"),
                "maps", "small_track.umdf.json"
            )
        ),
        "skidpad":
        Map.from_umdf(
            os.path.join(
                get_package_share_directory("fs_description"),
                "maps", "skidpad.umdf.json"
            )
        )
    }

    @staticmethod
    def get_maps():
        """
        returns list of available static and generated maps
        """
        options = list(StaticMaps.static_maps.values())
        return options

    @staticmethod
    def generate_static_map(name, track_width=5.0, cone_separation_distance=2.0):
        if name in StaticMaps.static_maps:
            return StaticMaps.static_maps[name]
