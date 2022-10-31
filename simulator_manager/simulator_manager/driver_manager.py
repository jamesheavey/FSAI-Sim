from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import os
import sys
import subprocess
import psutil
import re
import signal
import time
from threading import Thread
import yaml
import json
from fs_utils.file_system import touch_folder

ON_POSIX = 'posix' in sys.builtin_module_names


class DriverProcess:
    def __init__(self, package, launch_file, config_path, stdout_callback, stderr_callback):
        self.package = package
        self.launch_file = launch_file
        self.config_path = config_path
        self._process = None
        self._p = None
        self._stdout_thread = None
        self._stderr_thread = None
        self._stdout_callback = stdout_callback
        self._stderr_callback = stderr_callback

    def _output_handler(self, stream, callback):
        for line in iter(stream.readline, b''):
            callback(line.decode().rstrip())
        stream.close()

    def start(self):
        if self._process is not None:
            return
        self._p = subprocess.Popen(
            ["ros2", "launch", self.package, f"{self.launch_file}.launch.py", f"config_path:={self.config_path}"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=ON_POSIX
        )

        self._stdout_thread = Thread(target=self._output_handler, args=(self._p.stdout, self._stdout_callback, ))
        self._stdout_thread.daemon = True
        self._stdout_thread.start()
        self._stderr_thread = Thread(target=self._output_handler, args=(self._p.stderr, self._stderr_callback, ))
        self._stderr_thread.daemon = True
        self._stderr_thread.start()

        self._process = psutil.Process(self._p.pid)

    def stop(self):
        if self._process is None:
            return

        self._p.send_signal(signal.SIGINT)

        while self._p.poll():
            print("Waiting for process to terminate")
            time.sleep(0.5)

        self._p.terminate()

        # Murder the children
        try:
            children = self._process.children(recursive=True)
        except psutil.NoSuchProcess:
            return

        for p in [self._process] + list(children):
            try:
                p.kill()
            except (psutil.NoSuchProcess, psutil.psutil.AccessDenied):
                print("failed to kill process")

        self._process = None
        self._p = None

        # Cleanup listener threads
        self._stdout_thread.join()
        self._stdout_thread = None
        self._stderr_thread.join()
        self._stderr_thread = None

    def restart(self):
        if self._process is None:
            return
        self.stop()
        self.start()

    @property
    def signature(self):
        return f"{self.package}:{self.launch_file}"


class DriverManager:
    ignored_packages = [
        "vehicle_evaluator",
        "supervisor",
        "simulator_manager",
        "fs_msgs",
        "fs_description"
    ]

    def __init__(self, stdout_callback, stderr_callback):
        self.current_drivers = None
        self._stdout_callback = stdout_callback
        self._stderr_callback = stderr_callback
        self._config_dir = os.path.join(get_package_share_directory("simulator_manager"), "driver_config")
        touch_folder(self._config_dir)

    def get_possible_drivers(self):
        packages = [
            (package, path, get_package_share_directory(package))
            for package, path in get_packages_with_prefixes().items()
            if "/install/" in path and package not in self.ignored_packages]
        for package, path, share in packages:
            files = [
                match.group("launch_file") for f in os.listdir(share)
                if os.path.isfile(os.path.join(share, f)) and
                (match := re.match(r"^(?P<launch_file>.*)\.launch\.py$", f))
            ]
            for launch_file in files:
                yield package, launch_file

    def launch_drivers(self, drivers):
        if len(drivers) <= 0:
            raise Exception("You must select at least one driver launch file")
        if self.current_drivers is not None:
            raise Exception("Driver already running")

        all_drivers = list(self.get_possible_drivers())

        self.current_drivers = []
        for driver in drivers:
            if (driver.package, driver.launch_file) not in all_drivers:
                raise Exception("Driver not found")

            self.current_drivers.append(
                DriverProcess(driver.package, driver.launch_file, self._get_config_file(
                    driver), self._stdout_callback, self._stderr_callback)
            )
        for driver in self.current_drivers:
            driver.start()

    def _get_config_file(self, driver):
        path = os.path.join(self._config_dir, f"{driver.package}_{driver.launch_file}.yaml")
        with open(path, "w") as f:
            try:
                params = json.loads(driver.parameters)
            except json.JSONDecodeError:
                return path

            yaml.dump({
                key: {
                    "ros__parameters": value
                }
                for key, value in params.items()
            }, f)
        return path

    def stop_drivers(self):
        if self.current_drivers is None:
            raise Exception("Driver not running")

        for driver in self.current_drivers:
            driver.stop()
        self.current_drivers = None

    def restart_drivers(self):
        if self.current_drivers is None:
            raise Exception("Driver not running")

        for driver in self.current_drivers:
            driver.restart()

    @property
    def signature(self):
        if self.current_drivers is None:
            return ""
        return ','.join([driver.signature for driver in self.current_drivers])

    def stop_driver_if_running(self):
        if self.current_drivers is None:
            return
        self.stop_drivers()
