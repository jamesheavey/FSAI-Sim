from ament_index_python.packages import get_package_share_directory
from .state_abc import StateABC
from ..setup import Setup
import os
import cv2

TMP_ZIP = os.path.join(get_package_share_directory("supervisor"), "tmp.zip")


class Report:
    def __init__(self, report_msg):
        self.id = report_msg.id
        self.car = report_msg.car
        self.map = report_msg.map
        self.started = report_msg.started
        self.zip_path = report_msg.zip_path
        self.report_path = report_msg.report_path
        self.img_dir = report_msg.image_directory

    def to_dict(self):
        return {
            "id": self.id,
            "car": self.car,
            "map": self.map,
            "started": self.started
        }

    def get_frame_jpeg(self, frame_key, frame_id):
        if not os.path.isfile((path := os.path.join(self.img_dir, f"{frame_key}.avi"))):
            raise FileNotFoundError()
        cap = cv2.VideoCapture(path)
        if int(frame_id) > cap.get(cv2.CAP_PROP_FRAME_COUNT):
            raise EOFError()
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(frame_id) - 1)
        _, frame = cap.read()
        _, buffer = cv2.imencode('.jpg', frame)
        return buffer.tobytes()


class Reports(StateABC):

    def __init__(self, parent, ros):
        super().__init__(parent)
        self._reports = {}
        self._ros = ros
        self._ros.service_request(
            Setup.Services.VehicleEvaluator.get_reports,
            self._update_reports
        )
        self._ros.set_listener_lambda(
            Setup.Topics.Eval.report,
            lambda _, msg: self._new_report(msg.data)
        )

    def _update_reports(self, msg):
        self._reports = {
            report.id: Report(report)
            for report in msg.reports
        }
        self.state_changed()

    def _add_report(self, msg):
        self._ros.log.info(f"Report added: {msg.report.id}")
        report = msg.report
        if not report.id:
            return
        self._reports[report.id] = Report(report)
        self.state_changed()

    def _new_report(self, report_id):
        self._ros.log.info(f"new report: {report_id}")
        self._ros.service_request(
            Setup.Services.VehicleEvaluator.get_report,
            self._add_report,
            report_id
        )

    def _report_deleted(self, idx, resp):
        if not resp.success:
            return
        if idx not in self._reports:
            return
        del self._reports[idx]
        self.state_changed()

    def delete_file(self, idx):
        """
        Delete a file with idx
        """
        self._ros.service_request(
            Setup.Services.VehicleEvaluator.delete_report,
            lambda resp: self._report_deleted(idx, resp),
            idx
        )

    def upload_file(self, file):
        """
        Save file uploaded from API
        """
        if "/" in file.filename or ".." in file.filename:
            raise FileExistsError()
        if os.path.splitext(file.filename)[-1] != ".zip":
            raise FileExistsError()

        file.save(TMP_ZIP)

        self._ros.service_request(
            Setup.Services.VehicleEvaluator.add_report,
            self._add_report,
            TMP_ZIP
        )
        return ""

    def get_file(self, idx):
        """
        Load file with idx
        """

        if idx not in self._reports:
            raise FileNotFoundError()

        return self._reports[idx].report_path

    def get_frame_jpeg(self, idx, frame_key, frame_id):
        if idx not in self._reports:
            raise FileNotFoundError()
        return self._reports[idx].get_frame_jpeg(frame_key, frame_id)

    def get_zip(self, idx):
        if idx not in self._reports:
            raise FileNotFoundError()
        return self._reports[idx].zip_path

    def to_dict(self):
        return list(map(lambda r: r.to_dict(), self._reports.values()))
