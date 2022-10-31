import os
from ament_index_python.packages import get_package_share_directory

REPORTS_FOLDER = "reports"

REPORTS_PATH = os.path.join(
    get_package_share_directory("vehicle_evaluator"),
    REPORTS_FOLDER)

IMAGE_FOLDER = "images"

ZIP_TMP = os.path.join(
    get_package_share_directory("vehicle_evaluator"),
    "tmp.zip"
)
