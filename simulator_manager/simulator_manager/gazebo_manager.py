import subprocess
import psutil
import time

from fs_msgs.msg import Map


class GazeboStatus:
    unknown = -1
    not_running = 0
    healthy = 1
    error = 2


class GazeboProcess:
    def __init__(self, world, gui, timeout):
        self.world = world
        self.gui = gui
        self.timeout = timeout
        self.process = None

    def launch(self):
        p = subprocess.Popen(
            ["ros2", "launch", "simulator_manager", "gazebo.launch.py",
                f"world:={self.world.path}", f"gui:={str(self.gui)}"],
        )

        self.process = psutil.Process(p.pid)

        start_time = time.time()
        while not self.status == GazeboStatus.healthy and time.time() - start_time < self.timeout:
            time.sleep(0.1)

        if self.status != GazeboStatus.healthy:
            self._kill_process()
            raise TimeoutError()

    def _kill_process(self):
        process = self.process
        try:
            children = process.children(recursive=True)
        except psutil.NoSuchProcess:
            return

        all_processes = [process] + list(children)
        for p in all_processes:
            try:
                p.kill()
            except psutil.NoSuchProcess:
                pass
            except psutil.AccessDenied:
                pass

    def shutdown(self):
        self._kill_process()

    @property
    def status(self):
        if not self.process:
            return GazeboStatus.not_running

        children = self.process.children(recursive=True)
        names = list(map(lambda c: c.name(), children))
        services_available = "gzserver" in names and\
            (not self.gui or "gzclient" in names)

        if services_available:
            return GazeboStatus.healthy
        else:
            return GazeboStatus.error


class GazeboManager:
    default_timeout = 60

    def __init__(self):
        self.currently_running = None

    def killall(self):
        subprocess.run(
            ["killall", "gzserver", "gzclient"],
            capture_output=True
        )

    @property
    def current_status(self):
        if not self.currently_running:
            return GazeboStatus.not_running
        else:
            return self.currently_running.status

    @property
    def current_map(self):
        if not self.currently_running:
            return None
        else:
            return self.currently_running.world

    @property
    def current_map_msg(self):
        if not self.currently_running:
            return Map()
        else:
            return self.currently_running.world.to_msg()

    def launch(self, world, gui=True, timeout=-1):
        if timeout <= 0:
            timeout = GazeboManager.default_timeout
        if self.current_status != GazeboStatus.not_running:
            raise Exception("Gazebo is already running")

        self.killall()
        self.currently_running = GazeboProcess(world, gui, timeout)
        self.currently_running.launch()

    def shutdown(self):
        if self.current_status == GazeboStatus.not_running:
            raise Exception("Gazebo not running")

        self.currently_running.shutdown()

        self.currently_running = None
