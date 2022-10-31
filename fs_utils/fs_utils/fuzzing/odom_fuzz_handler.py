from .fuzz_handler_abc import FuzzHandlerABC
from fs_utils.conversions import quat_to_euler, euler_to_quat
import numpy as np
from geometry_msgs.msg import Quaternion


class OdomFuzzHandler (FuzzHandlerABC):
    """
    Fuzzer for odometry messages
    Apply enable/disable and noise
    """

    def __init__(self):
        self.last_pos = None
        self.last_ang = None
        self.last_fuzz_pos = None
        self.last_fuzz_ang = None
        self.start_pos = None
        self.start_pos_threshold = 0.1
        self.started = False

        self.pos_variance = 2e-1
        self.ang_variance = 2e-2
        self.odom_enabled = True
        self.fuzzing_enabled = False

        self.pos_sigma = np.power(self.pos_variance, 2)
        self.ang_sigma = np.power(self.ang_variance, 2)
        self.pos_covariance_mapping = {0: 0, 7: 0, 35: 1}
        self.ros_pos_covariance = [0 for _ in range(36)]    # Create covariance matrix
        self.update_covariance()

    def update_covariance(self):
        """
        Recalculate sigma and covariance
        """
        self.pos_sigma = np.power(self.pos_variance, 2)
        self.ang_sigma = np.power(self.ang_variance, 2)
        # Create sparse diagonal covariance matrix
        self.ros_pos_covariance = [
            float(0 if i not in self.pos_covariance_mapping else [
                  self.pos_variance, self.ang_variance][self.pos_covariance_mapping[i]])
            for i in range(36)]

    def set_config(self, config):
        """
        Handle config change
        """
        if "fuzzing_enabled" in config:
            self.fuzzing_enabled = config.get("fuzzing_enabled")
        if "odom_enabled" in config:
            self.odom_enabled = config.get("odom_enabled")
        if "pos_covariance" in config and "rot_covariance" in config:
            self.pos_variance = config.get("pos_covariance")
            self.ang_variance = config.get("rot_covariance")
            self.update_covariance()

    def get_config(self):
        return {
            "fuzzing_enabled": self.fuzzing_enabled,
            "odom_enabled": self.odom_enabled,
            "pos_covariance": self.pos_variance,
            "rot_covariance": self.ang_variance,
        }

    def handle(self, msg):
        """
        Handle odom fuzzing
        """
        euler = quat_to_euler(msg.pose.pose.orientation)
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        ang = euler["yaw"]

        if self.start_pos is None:
            self.start_pos = pos

        def store_unfuzzed():
            self.last_pos = pos
            self.last_ang = ang
            self.last_fuzz_pos = pos
            self.last_fuzz_ang = ang

        if not self.started:
            if np.linalg.norm(self.start_pos - pos) > self.start_pos_threshold:
                self.started = True
            else:
                store_unfuzzed()
                return msg

        if not self.fuzzing_enabled:
            store_unfuzzed()
            return msg
        if not self.odom_enabled:
            store_unfuzzed()
            return None

        # If previous frame is missing, don't fuzz
        if self.last_pos is None or self.last_fuzz_pos is None:
            store_unfuzzed()
            return msg

        # TODO: fuzz twist

        def get_cardinal_vectors(yaw):
            """
            Calculate rotation matrix and vehicle forward/right from yaw
            """
            rot_matrix = np.array([
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw), np.cos(yaw)],
            ])
            forward = np.matmul(
                rot_matrix,
                np.array([1, 0]))

            right = np.matmul(
                rot_matrix,
                np.array([0, 1]))
            return forward, right

        # Spacial fuzzing: calculate change in position relative to car
        # apply random to length, apply back to fuzzed angle
        prev_vehicle_forward, prev_vehicle_right = get_cardinal_vectors(self.last_ang)

        # Change in vehicle position
        d_pos = pos - self.last_pos

        # Get change in vehicle frame
        d_rel_pos = np.array([
            np.dot(d_pos, prev_vehicle_forward),
            np.dot(d_pos, prev_vehicle_right)
        ])

        fuzzed_prev_vehicle_forward, fuzzed_prev_vehicle_right = get_cardinal_vectors(self.last_fuzz_ang)

        # Apply random to change in pos (in car frame)
        d_rel_pos_len = np.linalg.norm(d_rel_pos)
        fuzzed_d_rel_pos = d_rel_pos * \
            np.abs(np.random.normal(d_rel_pos_len, self.pos_sigma)) * np.sign(d_rel_pos_len) / d_rel_pos_len

        # Update fuzzed position with fuzzed change in pos, covert change to world frame, using fuzzed cardinals
        fuzzed_pos = self.last_fuzz_pos + \
            fuzzed_d_rel_pos[0] * fuzzed_prev_vehicle_forward + \
            fuzzed_d_rel_pos[1] * fuzzed_prev_vehicle_right

        # Fuzz rotation
        d_ang = ang - self.last_ang
        fuzzed_ang = self.last_fuzz_ang + np.random.normal(d_ang, self.ang_sigma)

        # Store last values
        self.last_pos = pos
        self.last_ang = ang
        self.last_fuzz_pos = fuzzed_pos
        self.last_fuzz_ang = fuzzed_ang

        # Update message
        msg.pose.pose.position.x = fuzzed_pos[0]
        msg.pose.pose.position.y = fuzzed_pos[1]
        msg.pose.pose.orientation = Quaternion(**euler_to_quat(euler["roll"], euler["pitch"], fuzzed_ang))
        msg.pose.covariance = self.ros_pos_covariance
        return msg

    def reset(self):
        self.last_pos = None
        self.last_ang = None
        self.last_fuzz_pos = None
        self.last_fuzz_ang = None
        self.start_pos = None
        self.started = False
