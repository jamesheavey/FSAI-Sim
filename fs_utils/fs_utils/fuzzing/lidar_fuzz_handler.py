from .fuzz_handler_abc import FuzzHandlerABC
import numpy as np


class LidarFuzzHandler(FuzzHandlerABC):
    """
    Fuzzer for lidar messages
    Apply enable/disable and gaussian and salt & pepper noise
    """

    def __init__(self):
        self.gauss_configuration = {
            "enabled": True,
            "variance": 5e-1
        }
        self.snp_configuration = {
            "enabled": True,
            "salt_pepper_ratio": 0.5,
            "quantity": 0.01
        }
        self.lidar_enabled = True
        self.fuzzing_enabled = False

    def set_config(self, config):
        """
        Handle config change
        """
        if "fuzzing_enabled" in config:
            self.fuzzing_enabled = config.get("fuzzing_enabled")
        if "lidar_enabled" in config:
            self.lidar_enabled = config.get("lidar_enabled")
        if "gauss_config" in config:
            self.gauss_configuration = config.get("gauss_config")
        if "snp_config" in config:
            self.snp_configuration = config.get("snp_config")

    def get_config(self):
        return {
            "fuzzing_enabled": self.fuzzing_enabled,
            "lidar_enabled": self.lidar_enabled,
            "snp_config": self.snp_configuration,
            "gauss_config": self.gauss_configuration,
        }

    def noise_gauss(self, ranges, var):
        """
        Apply gaussian noise
        """
        return np.random.normal(ranges, var**2)

    def noise_salt_and_pepper(self, ranges, salt_pepper_ratio, amount):
        """
        Apply salt & pepper noise
        """
        def apply(ratio, val):
            num = np.ceil(amount * ranges.size * ratio)
            places = np.random.randint(0, ranges.size, int(num))
            ranges[places] = val
            return ranges

        ranges = apply(salt_pepper_ratio, np.inf)
        ranges = apply(1 - salt_pepper_ratio, 0)

        return ranges

    def handle(self, msg):
        """
        Handle laser scan (lidar) fuzzing
        """
        if not self.fuzzing_enabled:
            return msg
        if not self.lidar_enabled:
            return None

        range_min = msg.range_min
        range_max = msg.range_max
        ranges = np.array(msg.ranges)

        # Apply noises
        if self.snp_configuration.get("enabled", False):
            ranges = self.noise_salt_and_pepper(
                ranges,
                self.snp_configuration.get("salt_pepper_ratio", 0.5),
                self.snp_configuration.get("quantity", 0.05))

        if self.gauss_configuration.get("enabled", False):
            ranges = self.noise_gauss(
                ranges,
                self.gauss_configuration.get("variance", 1e-3))

        msg.ranges = np.clip(ranges, range_min, range_max).tolist()  # Enforce range min and max
        return msg
