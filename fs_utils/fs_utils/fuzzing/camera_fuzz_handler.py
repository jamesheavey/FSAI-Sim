from .fuzz_handler_abc import FuzzHandlerABC
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class CameraFuzzHandler(FuzzHandlerABC):
    """
    Fuzzer for camera messages
    Apply enable/disable and gaussian and salt & pepper noise
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.gauss_configuration = {
            "enabled": True,
            "variance": 1e-5
        }
        self.snp_configuration = {
            "enabled": True,
            "salt_pepper_ratio": 0.5,
            "quantity": 4e-3
        }
        self.camera_enabled = True
        self.fuzzing_enabled = False

    def set_config(self, config):
        """
        Handle config change
        """
        if "fuzzing_enabled" in config:
            self.fuzzing_enabled = config.get("fuzzing_enabled")
        if "camera_enabled" in config:
            self.camera_enabled = config.get("camera_enabled")
        if "gauss_config" in config:
            self.gauss_configuration = config.get("gauss_config")
        if "snp_config" in config:
            self.snp_configuration = config.get("snp_config")

    def get_config(self):
        return {
            "fuzzing_enabled": self.fuzzing_enabled,
            "camera_enabled": self.camera_enabled,
            "snp_config": self.snp_configuration,
            "gauss_config": self.gauss_configuration,
        }

    def noise_gauss(self, frame, var=1e-5):
        """
        Apply gaussian noise to frame
        ref: https://stackoverflow.com/a/30609854/4441404
        """
        row, col, ch = frame.shape
        mean = 0.
        sigma = var**0.5
        gauss = np.random.normal(mean, sigma, (row, col, ch))
        gauss = gauss.reshape(row, col, ch)
        noisy = frame + gauss
        return noisy

    def noise_salt_and_pepper(self, frame, salt_pepper_ratio=0.5, amount=0.004):
        """
        Apply salt & pepper noise to frame
        ref: https://stackoverflow.com/a/30609854/4441404
        """
        noisy = np.copy(frame)

        def apply(ratio, val):
            num = np.ceil(amount * frame.size * ratio)
            coords = [np.random.randint(0, i - 1, int(num)) for i in frame.shape]
            noisy[coords] = val
            return noisy

        # Salt mode
        noisy = apply(salt_pepper_ratio, 1)

        # Pepper mode
        noisy = apply(1 - salt_pepper_ratio, 0)

        return noisy

    def to_float(self, frame):
        return frame.astype(np.float32) / 255.

    def to_uint8(self, frame):
        return (frame * 255.).astype(np.uint8)

    def handle(self, msg):
        """
        Handle image frame fuzzing
        """
        # If fuzzing disabled return the same frame
        if not self.fuzzing_enabled:
            return msg
        # If camera disabled return nothing
        if not self.camera_enabled:
            return None

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return msg

        frame = self.to_float(frame)    # Convert to 0-1 float image (easier to add noise)

        # Apply noises
        if self.snp_configuration.get("enabled", False):
            frame = self.noise_salt_and_pepper(
                frame,
                salt_pepper_ratio=self.snp_configuration.get("salt_pepper_ratio", 0.5),
                amount=self.snp_configuration.get("quantity", 4e-3))

        if self.gauss_configuration.get("enabled", False):
            frame = self.noise_gauss(
                frame,
                var=self.gauss_configuration.get("variance", 1e-5))

        try:
            frame = self.to_uint8(frame)    # Convert back to 0-255 uint image
            return self.bridge.cv2_to_imgmsg(frame, "bgr8")
        except CvBridgeError:
            return msg      # Fall back to returning the original
