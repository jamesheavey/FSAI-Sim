import xml.etree.ElementTree as ET
from matplotlib import pyplot as plt
import numpy as np
import json


class Umdf:
    """
    Class encapsulating all the info to create a universal map description file
    """

    def __init__(self, cones, centre_line, start, angle):
        # Cones dict to separate properties
        self.blue_cones, self.yellow_cones, self.big_cones, self.orange_cones = [
            cones.get(key, []) for key in ["blue", "yellow", "big", "orange"]
        ]
        self.centre_line = centre_line
        self.start = start
        self.start_angle = angle

    def save(self, world_file, name, out_path):
        """
        Save Umdf json file, add world_file and name, output to out_path
        """
        # Helper functions
        def np_to_dict(a): return {"x": a[0], "y": a[1]}
        def np_to_dict_all(cones): return list(map(np_to_dict, cones))

        with open(out_path, "w") as f:
            json.dump({
                "name": name,
                "path": world_file,
                "start": {
                    "position": {"x": self.start[0], "y": self.start[1]},
                    "rotation": {"yaw": self.start_angle}
                },
                "blue_cones": np_to_dict_all(self.blue_cones),
                "yellow_cones": np_to_dict_all(self.yellow_cones),
                "orange_cones": np_to_dict_all(self.big_cones + self.orange_cones),
                "centreline": np_to_dict_all(self.centre_line),
            }, f, indent=4)

    def show(self):
        """
        Display generated map using matplotlib
        """
        def plot(points, style):
            x, y = list(zip(*points))
            plt.plot(x, y, style)

        plot(self.blue_cones, "bo")
        plot(self.yellow_cones, "yo")
        plot(self.big_cones + self.orange_cones, "ro")
        plot([self.start], "gx")
        plot(self.centre_line, "x-")
        print(self.start_angle)

        plt.show()


CONES = {
    "model://blue_cone": "blue",
    "model://yellow_cone": "yellow",
    "model://big_cone": "big",
    "model://orange_cone": "orange"
}


def get_cones(path):
    """
    Extract cones from xml
    """
    tree = ET.parse(path)
    root = tree.getroot()
    cones = {}
    for e in root.find("model").findall("include"):
        cone_type = e.find("uri").text
        pose_raw = np.array(list(map(float, e.find("pose").text.split())))[:2]
        cones[CONES[cone_type]] = cones.get(CONES[cone_type], []) + [pose_raw]
    return cones


def find_nearest(this, other):
    """
    Given to lists of coordinates, for each in this list find the nearest in the other list
    """
    for cone in this:
        # Find nearest cone
        min_o = None
        for other_cone in other:
            # If no nearest cone, next cone is the nearest
            # Otherwise check next if cone is closer than the current nearest
            if min_o is None or\
                    np.linalg.norm(cone - min_o) > np.linalg.norm(cone - other_cone):
                min_o = other_cone
        # Return pairs
        yield avg(cone, min_o)


def avg(a, b): return (a + b) / 2   # Average helper function


def find_start(cones, blues):
    """
    Find the start position and forward, based on a set of 4 orange cones.
    Forward is determined by the side of the blue cones
    """
    all_cones = list(cones)     # Copy cones
    current = all_cones.pop()   # Take the first one
    p_i, pair = min(enumerate(all_cones),
                    key=lambda c: np.linalg.norm(current - c[1]))       # Find the cone nearest to the first
    all_cones.pop(p_i)          # Remove second cone
    # Two sides are the two selected and the other two
    sides = ([current, pair], all_cones)

    centres = list(map(lambda s: avg(*s), sides))   # Average both sides

    centre = avg(*centres)  # Find the centre point between the two sides

    # Select the side closest to blue cones
    blue_centre = min(centres,
                      key=lambda c: min(map(lambda b: np.linalg.norm(c - b), blues)))

    # Set forward as the 90 degrees rotation of the centre pointing to the cones on the blue side
    forward = np.matmul(np.array([[0, 1], [-1, 0]]), blue_centre - centre)

    # Return centre and normalised forward
    return centre, forward / np.linalg.norm(forward)


def sort_centre_line(points, start, forward):
    """
    Given a set of points on the centre line, and a starting point with a forward,
    order the points such that always the next closest is selected that is in front of the last
    """
    points = list(points)
    current = start
    fwd = forward
    while len(points) > 0:
        # Find a point that is in front of current, and is closest to it
        n_i, nxt = min(
            filter(
                lambda c: np.dot(c[1] - current, fwd) > 0,
                enumerate(points)),
            key=lambda c: np.linalg.norm(c[1] - current))
        # Remove the selected point
        points.pop(n_i)
        yield nxt
        # Calculate new forward and set new current
        fwd = nxt - current
        current = nxt


def process_regular_map(path):
    """
    Assuming a standard map with a single circuit, build a map
    """
    cones = get_cones(path)

    # Find points between cones
    pairs = list(find_nearest(cones["blue"], cones["yellow"])) + \
        list(find_nearest(cones["yellow"], cones["blue"]))

    # Remove duplicates
    pairs = np.unique(pairs, axis=0)

    # Find start
    centre, forward = find_start(cones["big"], cones["blue"])

    # Find centre line
    centre_line = [centre] + list(sort_centre_line(pairs, centre, forward))

    # Find car starting angle
    angle = np.arctan2(forward[1], forward[0])

    return Umdf(cones, centre_line, centre, angle)


def process_skidpad_map(path):
    """
    Special map builder only for the skidpad, as that is special
    """
    # Ugly hacky helper function to find certain parts of the skidpad
    def select_bottom_cones(cones):
        return list(filter(lambda c: c[1] < 0, cones))

    def select_top_cones(cones):
        return list(filter(lambda c: 20 > c[1] > 0, cones))

    def select_left_circle(cones):
        return list(filter(lambda c: c[0] < 0, cones))

    def select_right_circle(cones):
        return list(filter(lambda c: c[0] > 0.1, cones))

    def select_centre_points(cones):
        return list(filter(lambda c: -0.1 < c[0] < 0.1, cones))

    cones = get_cones(path)

    # Find points between cones
    pairs = list(find_nearest(cones["blue"], cones["yellow"])) + \
        list(find_nearest(cones["yellow"], cones["blue"]))

    # Deduplicate points
    pairs = np.unique(pairs, axis=0)

    # Find top orange cones that is the end of the track
    top_orange_cones = select_top_cones(cones["orange"])

    top_pairs = list(find_nearest(select_left_circle(
        top_orange_cones), select_right_circle(top_orange_cones)))

    # Find start poisition using the bottom orange cones, and using yellow instead of blue, because this track is dumb
    centre, forward = find_start(
        select_bottom_cones(cones["orange"]), cones["yellow"])

    # Find points on various parts of the track
    centre_points = select_centre_points(pairs)
    right_circle = select_right_circle(pairs)
    left_circle = select_left_circle(pairs)

    # Section from start to middle
    centre_line = [centre] + \
        list(sort_centre_line(centre_points, centre, forward))

    # Section from middle through the right circle
    centre_line += list(sort_centre_line(right_circle,
                                         centre_line[-1], forward))
    # Section from end of right circle through middle again
    centre_line += list(sort_centre_line(centre_points,
                                         centre_line[-1], forward))
    # TODO: if needed add second loop here
    # Section from middle through the left circle
    centre_line += list(sort_centre_line(left_circle,
                                         centre_line[-1], forward))
    # Section from end of left circle through middle again
    centre_line += list(sort_centre_line(centre_points,
                                         centre_line[-1], forward))
    # Section from middle through to the end
    centre_line += list(sort_centre_line(top_pairs, centre_line[-1], forward))

    # Calculate vehicle start angle
    angle = np.arctan2(forward[1], forward[0])

    return Umdf(cones, centre_line, centre, angle)


def convert_small_track():
    umd = process_regular_map(
        "/home/ros/fs_ws/src/f21ai-sim/fs_description/models/small_track/model.sdf"
    )
    umd.save(
        "worlds/small_track.world",
        "small_track",
        "/home/ros/fs_ws/src/f21ai-sim/fs_description/maps/small_track.umdf.json"
    )
    umd.show()


def convert_skidpad():
    umd = process_skidpad_map(
        "/home/ros/fs_ws/src/f21ai-sim/fs_description/models/skidpad/model.sdf"
    )
    umd.save(
        "worlds/skidpad.world",
        "skidpad",
        "/home/ros/fs_ws/src/f21ai-sim/fs_description/maps/skidpad.umdf.json"
    )
    umd.show()


if __name__ == "__main__":
    convert_skidpad()
    # convert_small_track()
