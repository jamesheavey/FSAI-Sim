import json
import os
from .constants import REPORTS_PATH, IMAGE_FOLDER, ZIP_TMP
import shutil


class Report:
    def __init__(self, idx):
        if not idx:
            raise FileNotFoundError(f"Couldn't find report {idx}")
        self.id = idx
        self._dir = os.path.join(REPORTS_PATH, idx)
        self._report_path = os.path.join(self._dir, f"{idx}.json")
        if not os.path.isdir(self._dir) or not os.path.isfile(self._report_path):
            raise FileNotFoundError(f"Couldn't find report {idx}")
        self._zip_path = os.path.join(self._dir, f"{idx}.zip")
        self._report = None
        self._deleted = False

    @property
    def report(self):
        if self._deleted:
            return None
        if self._report is not None:
            return self._report
        with open(self._report_path) as f:
            self._report = json.load(f)
        return self._report

    @property
    def zip(self):
        if self._deleted:
            return None
        if os.path.isfile(self._zip_path):
            return self._zip_path
        shutil.make_archive(
            ZIP_TMP[:-4],
            "zip",
            root_dir=REPORTS_PATH,
            base_dir=self.id
        )
        shutil.move(ZIP_TMP, self._zip_path)
        return self._zip_path

    @property
    def img_dir(self):
        if self._deleted:
            return None
        return os.path.join(self._dir, IMAGE_FOLDER)

    @property
    def report_path(self):
        return self._report_path

    def delete(self):
        if self._deleted:
            return
        shutil.rmtree(self._dir)
        self._deleted = True
