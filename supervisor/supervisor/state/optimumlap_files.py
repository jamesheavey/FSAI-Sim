from ament_index_python.packages import get_package_share_directory
from .state_abc import StateABC
import os


class OptimumlapFiles(StateABC):
    """
    Class for managing optimumlap files
    """
    folder_path = "maps/optimumlap"
    extentions = [".OLTra", ".csv"]

    def __init__(self, parent):
        super().__init__(parent)
        self._path = os.path.join(get_package_share_directory("supervisor"), self.folder_path)
        self._touch_folder()
        self._files = self._scan_files()

    def _touch_folder(self):
        """
        Create target folder if not already exist
        """
        if not os.path.isdir(self._path):
            os.makedirs(self._path)

    def _has_extension(self, filename):
        return any(os.path.splitext(filename)[-1] == extension for extension in self.extentions)

    def _scan_files(self):
        """
        Discover all files in the target folder
        """
        return [f
                for f in os.listdir(self._path)
                if os.path.isfile(os.path.join(self._path, f)) and self._has_extension(f)]

    def update_files(self):
        """
        Update the list of files available and dispatch state changed
        """
        self._files = self._scan_files()
        self.state_changed()
        return self._files

    def _file_exists(self, filename):
        """
        Check if a file with filename exists
        """
        return os.path.isfile(os.path.join(self._path, filename))

    def delete_file(self, filename):
        """
        Delete a file with filename
        """
        # Make some attempt at sanitizing the filename
        if "/" in filename or ".." in filename:
            return False
        if not self._file_exists(filename):
            return False
        os.remove(os.path.join(self._path, filename))
        self.update_files()

    def upload_file(self, filename, file):
        """
        Save uploaded file object
        """
        # Make some attempt at sanitizing the filename
        if "/" in filename or ".." in filename:
            return False
        if not self._has_extension(filename):
            return False
        file.save(os.path.join(self._path, filename))
        self.update_files()

    def get_full_path(self, filename):
        if not self._file_exists(filename):
            raise FileNotFoundError(f"Couldn't find file {filename}")
        return os.path.join(self._path, filename)

    def to_dict(self):
        """
        Return the list of files
        TODO: at some point they should be populated as map objects
        """
        return self._files
