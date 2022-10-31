import math
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from .optimumlap_parser import OptimumlapTrack
from .generate_map import GenerateMap


class OptimumLapMap:
    """
    Class to extract centerline from optimumlap track data in multiple formats
    """

    @staticmethod
    def get_map(map_path, track_width, cone_separation_distance):
        """
        returns map object for requested track
        """
        map_name = os.path.basename(map_path)
        return OptimumLapMap.generate_optimumlap_from_file(map_name,  # generates selected optimumlap track
                                                           map_path,
                                                           track_width,
                                                           start_position=(0.0, 0.0),
                                                           start_angle=0.0,
                                                           cone_separation_distance=2)

    @staticmethod
    def text_to_centerline(path):
        """
        Takes x and y coordinate data from an optimumlap export in .txt format and returns it as a centerline
        """
        path = os.path.join(get_package_share_directory("fs_description"),
                            "maps", "OptimumLap/Lime_rock_Mazda_mx-5.txt")
        centerline = []
        with open(path, 'r') as f:
            data = f.readlines()
        for line in data:  # loops through lines in text file
            line_sequence = line.split()  # split lines into columns
            x, y = line_sequence[0], line_sequence[1]  # assign corresponding coordinates to x and y
            centerline.append({  # append x and y coordinates
                "x": float(x),
                "y": float(y),
            })
        return centerline

    @staticmethod
    def generate_optimumlap_from_file(map_name, path_to_track_file, track_width,
                                      cone_separation_distance, start_position=(0.0, 0.0), start_angle=0.0):
        """
        Generates a track from the optimumlap track file
        """
        track = OptimumlapTrack(path_to_track_file)  # find optimumlap track object from file
        # generate centerline from track file
        centerline = OptimumLapMap.generate_centerline_from_sections(
            track, start_position[0], start_position[1], start_angle)
        # generate cones from centerline
        map = GenerateMap.generate_map(map_name, centerline, track_width, cone_separation_distance)
        return map

    @staticmethod
    def generate_centerline_from_sections(track_file, x=0.0, y=0.0, angle=0.0):
        """
        Loops through all track sections in the file and returns them in a centerline format
        """
        centerline = []
        track = track_file
        sections = track.sections  # extract sections from track file
        for section in sections:  # loop through sections
            # calculate centerline from section
            centerline_section, x, y, angle = OptimumLapMap.add_section_to_centerline(section, x, y, angle)
            centerline += centerline_section  # append centerline section to centerline
        return centerline

    @staticmethod
    def add_section_to_centerline(section, x, y, angle):
        """
        Takes a single section and converts it to a centerline section
        """
        length = section.length
        radius = section.corner_radius
        direction = section.corner_direction
        centerline = []
        if direction is None:
            for p in np.linspace(0, length, int(length * 10), endpoint=False):
                centerline.append({
                    "x": x + (p * math.cos(angle)),
                    "y": y + (p * math.sin(angle))
                })
            return centerline, x + length * math.cos(angle), y + length * math.sin(angle), angle
        else:
            corner_center_x = x + (radius * math.cos(angle + (direction * math.pi/2.0)))
            corner_center_y = y + (radius * math.sin(angle + (direction * math.pi/2.0)))
            rotation_angle = direction * length/radius
            start_angle = angle - (direction * (math.pi/2.0))
            end_angle = start_angle + rotation_angle
            for theta in np.linspace(start_angle, end_angle, int(length * 10), endpoint=False):
                centerline.append({
                    "x": corner_center_x + (radius * math.cos(theta)),
                    "y": corner_center_y + (radius * math.sin(theta)),
                })
            return (centerline,
                    corner_center_x + radius * math.cos(end_angle),
                    corner_center_y + radius * math.sin(end_angle),
                    angle + rotation_angle)

    @ staticmethod
    def generate_optimumlap_from_text(file_path, track_width=5.0, cone_separation_distance=2.0):
        centerline = []
        centerline = OptimumLapMap.text_to_centerline(file_path)
        map = GenerateMap.generate_map(centerline, track_width, cone_separation_distance)
        return map
