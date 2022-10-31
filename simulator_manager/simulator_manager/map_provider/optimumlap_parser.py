import os


def key_to_length(dt):
    return sum([v * (256 ** i) for i, v in enumerate(dt[0:4])])


def parse_track_section(data):
    def track_section(dt):
        val = key_to_length(dt)
        sections = [dt[4 + i * 8: 12 + i * 8] for i in range(val)]
        return sections

    def invert_track(dt):
        return [v for d in reversed(dt) for v in [(d // 16) % 16, d % 16]]

    def track_to_num(dt):
        inv = invert_track(dt)

        if all(i == 0 for i in inv):
            return None

        if inv == [11, 15, 15, *([0] * 13)]:
            return -1.0

        if not (inv[0] == 4 and inv[1] == 0) and not (inv[0] == 3):
            raise Exception("Invalid number format")

        base = 2 ** (inv[0] - 3 + inv[2] - inv[1])
        step = sum([base / (16 ** (1 + i)) * v for i, v in enumerate(inv[3:])])
        return base + step

    track_straights = track_section(data)

    num_straights = [track_to_num(straight) for straight in track_straights]

    return num_straights


def circuit_looping_type(data):
    if b"Open Circuit" in data:
        return False
    if b"Closed Circuit" in data:
        return True
    raise Exception("Couldn't determine track looping type")


def parse(data):
    index = {}
    for i, _ in enumerate(data):
        sq = tuple(data[i:i+4])
        if sq not in index:
            index[sq] = []
        index[sq] = index[sq] + [i]
    relevant = {k: v for k, v in index.items() if len(v) >= 3 and k[0] != 0}

    def find_with_offset(r):
        found = []
        for k, v in r.items():
            section_count = key_to_length(k)
            memsize = section_count * 8 + 4
            places = sorted(v)

            relevant_places = [p for p in places
                               if p + memsize in places or p - memsize in places]

            if len(relevant_places) >= 3:
                found.append((section_count, relevant_places))

        return found

    all_found = find_with_offset(relevant)

    has_nums = {}
    for section_count, places in all_found:
        sections = []
        for place in places:
            try:
                sections.append(parse_track_section(data[place:]))
            except Exception:
                pass
        if len(sections) == 3:
            has_nums[section_count] = sections

    if len(has_nums) != 1:
        raise Exception("Could not decode track")

    sections = list(zip(*list(has_nums.values())[0]))

    looping = circuit_looping_type(data)

    return sections, looping


def load_track(filename):
    with open(filename, "rb") as f:
        data = f.read()

    return parse(data)


class OptimumlapTrackSection:
    """
    Description of a single section of OptimumLap track
    """

    def __init__(self, section):
        self.length = section[0]
        self.corner_radius = section[1]
        self.corner_direction = section[2]


class OptimumlapTrack:
    """
    Track description generated from an OptimumLap track file
    """

    def __init__(self, filename):
        self.filename = os.path.basename(filename)
        sections, looping = load_track(filename)

        self.looping = looping
        self.sections = list(map(OptimumlapTrackSection, sections))


"""
Example usage:
from .optimumlap_parser import OptimumlapTrack

def load_track(path_to_track_file):
    # Check if file exists and if it has the right extension

    track = OptimumlapTrack(path_to_track_file)

    name = track.filename # (maybe strip out the extension)
    is_looping = track.looping

    sections = track.sections   # Array of sections

    example_section = sections[0]
    length_meters = example_section.length
    corner_radius_meters = example_section.corner_radius  # None if it's a straight
    corner_direction = example_section.corner_direction  # 1 if left, -1 if right, None if straight
    # Convert to centerline at will
"""
