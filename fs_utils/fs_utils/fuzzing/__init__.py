from rclpy.node import Node
from .fuzz_handler_abc import FuzzHandlerABC
from .odom_fuzz_handler import OdomFuzzHandler
from .camera_fuzz_handler import CameraFuzzHandler
from .lidar_fuzz_handler import LidarFuzzHandler
from fs_msgs.srv import GetFuzzingConfiguration, SetFuzzingConfiguration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from rclpy.qos import qos_profile_sensor_data
import json
import traceback


class Fuzzer:
    """
    Class description of fuzzing, take in and out topics and fuzz handler
    When there's a message on the in topic apply the fuzz handler and dispatch on out topic. Keep QOS the same
    """

    def __init__(self, node, sub_type, topic_in, topic_out, fuzz_handler, qos=1):
        assert isinstance(fuzz_handler, FuzzHandlerABC)
        self._handler = fuzz_handler

        node.create_subscription(sub_type, topic_in, self.handle, qos)
        self._publisher = node.create_publisher(sub_type, topic_out, qos)

    def handle(self, msg):
        if (resp := self._handler.handle(msg)):
            self._publisher.publish(resp)


class Fuzzing(Node):
    """
    Node to handle fuzzing all the topics
    """

    def __init__(self):
        super().__init__("fuzzer")

        self.handlers = {
            "odom": OdomFuzzHandler(),
            "camera1": CameraFuzzHandler(),
            "camera2": CameraFuzzHandler(),
            "camera0": CameraFuzzHandler(),
            "lidar": LidarFuzzHandler(),
        }

        self.fuzzers = (
            Fuzzer(self, Odometry, "/car/odom", "/car/odom/fuzzed",
                   self.handlers["odom"]),
            Fuzzer(self, Image, "/camera1/image_raw", "/camera1/image_fuzzed",
                   self.handlers["camera1"],
                   qos=qos_profile_sensor_data),
            Fuzzer(self, Image, "/camera2/image_raw", "/camera2/image_fuzzed",
                   self.handlers["camera2"],
                   qos=qos_profile_sensor_data),
            Fuzzer(self, Image, "/camera0/image_raw", "/camera0/image_fuzzed",
                   self.handlers["camera0"],
                   qos=qos_profile_sensor_data),
            Fuzzer(self, LaserScan, "/lidar", "/lidar/fuzzed",
                   self.handlers["lidar"]),
        )

        # Configuration services
        self.create_service(GetFuzzingConfiguration, "/fuzzing/get_config", self._get_config)
        self.create_service(SetFuzzingConfiguration, "/fuzzing/set_config", self._set_config)
        self.create_service(Empty, "/fuzzing/reset", self._reset_fuzzing)

    def _get_config(self, request, response):
        """
        Get config service handler
        """
        response.config_json = json.dumps({key: item.get_config()
                                           for key, item in self.handlers.items()})
        return response

    def _set_config(self, request, response):
        """
        Set config service handler
        """
        try:
            config = json.loads(request.config_json)
            for key, item in self.handlers.items():
                item.set_config(config.get(key, {}))
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to set config: {e}\n{traceback.format_exc()}")
            response.success = False
        return response

    def _reset_fuzzing(self, request, response):
        """
        Reset fuzzing service handler
        """
        for item in self.handlers.values():
            item.reset()
        return response
