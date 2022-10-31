import os
import json
from fs_utils.file_system import touch_folder, touch_json
from fs_msgs.msg import Report as ReportMsg
import shutil
from .constants import REPORTS_PATH
from .report import Report
import traceback


class Reports:
    index_file = "index.json"

    def __init__(self, node):
        self._node = node

        self._index_path = os.path.join(REPORTS_PATH, self.index_file)

        touch_folder(REPORTS_PATH)
        touch_json(self._index_path)

        self._currently_recording = None

        self._index = self._load_index()
        self._update_index()

    def _get_report(self, idx):
        try:
            return Report(idx)
        except FileNotFoundError:
            return None

    def _scan_directories(self):
        return {f: report
                for f in os.listdir(REPORTS_PATH)
                if os.path.isdir(os.path.join(REPORTS_PATH, f)) and (report := self._get_report(f)) and
                f != self._currently_recording}

    def _delete_directory(self, idx):
        dir_path = os.path.join(REPORTS_PATH, idx)
        if not os.path.isdir(dir_path):
            return
        shutil.rmtree(dir_path)

    def _load_index(self):
        if not os.path.isfile(self._index_path):
            return {}
        try:
            with open(self._index_path) as f:
                return json.load(f)
        except json.JSONDecodeError:
            return {}

    def _update_index(self):
        reports = self._scan_directories()

        for key in [key for key in self._index if key not in reports]:
            self._index.pop(key)
            self._delete_directory(key)

        for idx, report in [(idx, report) for idx, report in reports.items() if idx not in self._index]:
            self._index[idx] = {
                "id": idx,
                "started": report.report["started"],
                "map": report.report.get("map", {}).get("name", ""),
                "car": report.report.get("car", {}).get("name", ""),
                "zip_path": report.zip,
                "report_path": report.report_path,
                "img_dir": report.img_dir
            }

        with open(self._index_path, "w") as f:
            json.dump(self._index, f)

        return self._index

    def _report_from_index(self, idx):
        if not (report := self._index.get(idx, None)):
            return None
        return ReportMsg(
            id=str(idx),
            car=str(report.get("car", "")),
            map=str(report.get("map", "")),
            started=float(report.get("started", 0)),
            zip_path=str(report.get("zip_path", "")),
            report_path=str(report.get("report_path", "")),
            image_directory=str(report.get("img_dir", ""))
        )

    def get_report(self, request, response):
        try:
            self._index = self._update_index()
            idx = request.id
            if (report_msg := self._report_from_index(idx)) is None:
                return response
            response.report = report_msg
        except Exception as e:
            self._node.get_logger().error(f"Failed to get report ({request}): {e}\n{traceback.format_exc()}")
        return response

    def get_reports(self, reqeust, response):
        try:
            reports = [report for idx in self._index if (report := self._report_from_index(idx)) is not None]
            response.reports = reports
        except Exception as e:
            self._node.get_logger().error(f"Failed to launch map: {e}\n{traceback.format_exc()}")
        return response

    def add_report(self, request, response):
        try:
            path = request.zip_path
            if not os.path.isfile(path):
                return response
            old_index = dict(self._update_index())
            shutil.unpack_archive(path, REPORTS_PATH)
            self._index = self._update_index()
            new_keys = [key for key in self._index if key not in old_index]
            if len(new_keys) != 1:
                return response
            response.report = self._report_from_index(new_keys[0])
        except Exception as e:
            self._node.get_logger().error(f"Failed to add report ({request}): {e}\n{traceback.format_exc()}")
        return response

    def delete_report(self, request, response):
        try:
            idx = request.id

            if (report_dir := self._get_report(idx)) is not None:
                report_dir.delete()
            self._delete_directory(idx)
            self._index = self._update_index()
            response.success = True
        except Exception as e:
            response.success = False
            self._node.get_logger().error(f"Failed to delete report ({request}): {e}\n{traceback.format_exc()}")
        return response

    def set_current_report(self, idx):
        if idx is None and self._currently_recording is not None:
            self._index = self._update_index()

        self._currently_recording = idx
