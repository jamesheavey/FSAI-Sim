import time
from uuid import uuid1
from fs_utils.conversions import car_to_dict
import os
import cv2
import json
from ..reports.constants import REPORTS_PATH, IMAGE_FOLDER


class VideoFeed:
    """
    Single video feed writer
    Count the number of frames used
    """
    def __init__(self, writer):
        self._writer = writer
        self._next_frame = 0

    def write(self, frame):
        self._writer.write(frame)
        self._next_frame += 1

    @property
    def next_frame(self):
        return self._next_frame

    def release(self):
        self._writer.release()


class VideoHandler:
    """
    Handles video feeds during a simulation
    Provides unique feed ids and stores frames by key
    Must be flushed
    """
    def __init__(self, frame_rate):
        self._feeds = {}
        self._frame_rate = frame_rate
        self._fourcc = cv2.VideoWriter_fourcc(*'MPEG')

    def _get_feed(self, key, img_dir, frame):
        if key not in self._feeds:
            writer = cv2.VideoWriter(os.path.join(img_dir, f"{key}.avi"),
                                     self._fourcc, self._frame_rate, frame.shape[:2])
            self._feeds[key] = VideoFeed(writer)

        return self._feeds[key]

    def add_frame(self, key, img_dir, frame):
        feed = self._get_feed(key, img_dir, frame)
        feed.write(frame)

    def next_frame_id(self, key):
        if key in self._feeds:
            return self._feeds[key].next_frame
        return 0

    def flush(self):
        for feed in self._feeds.values():
            feed.release()
        self._feeds = {}


class EvaluationSnapshot:
    """
    Single snapshot of data in an evaluation report
    De-duplicates data keys
    """
    def __init__(self, t, video_handler):
        self._data = {}
        self._images = {}
        self._t = t
        self._video_handler = video_handler

    def add_data(self, key, data):
        self._data[key] = {"key": key, "data": data}

    def add_image(self, key, data, frame):
        frame_id = self._video_handler.next_frame_id(key)
        self._data[key] = {
            "key": key,
            "data": {**data, "frame_id": frame_id, "frame_key": key},
        }
        self._images[key] = frame

    def save_images(self, img_dir):
        for key, frame in self._images.items():
            self._video_handler.add_frame(key, img_dir, frame)
            # cv2.imwrite(os.path.join(img_dir, f"{frame_id}.png"), frame)
        self._images = {}

    def to_dict(self):
        return {"time": self._t, "data": list(self._data.values())}


class EvaluationReport:
    """
    Contain the reported snapshots of a simulation run
    """
    @staticmethod
    def gen_id(segments=1):
        return ''.join(str(uuid1()).split("-")[:segments])

    @staticmethod
    def get_frameid(img_dir, segments=3):
        frame_id = EvaluationReport.gen_id(segments)
        if os.path.isfile(os.path.join(img_dir, f"{frame_id}.png")):
            return EvaluationReport.get_frameid(img_dir=img_dir, segments=segments)
        return frame_id

    def __init__(self, map, car, idx, t=0, resolution=0.1):
        self._snapshots = []
        self._video_handler = VideoHandler(1/resolution)
        self.latest_snapshot = EvaluationSnapshot(t, self._video_handler)
        self._start = time.time()
        self._map = map
        self.id = idx
        self._car = car_to_dict(car)

        self._dir = os.path.join(REPORTS_PATH, self.id)
        os.mkdir(self._dir)
        self._img_dir = os.path.join(self._dir, IMAGE_FOLDER)
        os.mkdir(self._img_dir)

        self._report_path = os.path.join(self._dir, f"{self.id}.json")

    def add_data(self, key, data):
        self.latest_snapshot.add_data(key, data)

    def add_image(self, key, data, frame):
        # frame_id = self.get_frameid(self._img_dir)
        size = list(frame.shape)
        frame = cv2.resize(frame, (800, 800))
        data = {**data, "original_size": size}
        self.latest_snapshot.add_image(key, data, frame)

    def take_snapshot(self, t):
        self.latest_snapshot.save_images(self._img_dir)
        self._snapshots.append(self.latest_snapshot)
        self.latest_snapshot = EvaluationSnapshot(t, self._video_handler)

    def to_dict(self):
        return {
            "id": self.id,
            "started": self._start,
            "map": self._map.to_dict(),
            "car": self._car,
            "snapshots": list(map(lambda snapshot: snapshot.to_dict(), self._snapshots + [self.latest_snapshot]))
        }

    def save(self):
        with open(self._report_path, "w") as f:
            json.dump(self.to_dict(), f)
        self._video_handler.flush()
