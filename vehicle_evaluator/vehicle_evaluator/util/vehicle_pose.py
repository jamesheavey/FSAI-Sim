from fs_utils.conversions import quat_to_euler, transform_to_matrix
import numpy as np

from .derivative import Derivative
from .windowed_average import WindowedAverage
from .vehicle_vec import VehicleVec


class VehiclePose:
    """
    Hold all the information about the pose of the vehicle
    """

    def __init__(self,
                 root="odom_frame",
                 body="chassis",
                 wheels=["right_wheel", "left_wheel",
                         "right_wheel_rear", "left_wheel_rear"],
                 wheel_hinges={"right": "right_wheel_assembly", "left": "left_wheel_assembly"},
                 averaging_window_size=50,
                 pose_change_callbacks=[]):
        self._last_tfs = {}
        self._parents = {}
        self._root_frame = root
        self._body_frame = body
        self._wheel_frames = wheels
        self._hinge_frames = wheel_hinges

        self._pose_change_callbacks = pose_change_callbacks

        # Calculate velocity and acceleration. Apply a windowed average to make the data smoother
        self._velocity = Derivative(initial_value=np.array([0, 0, 0]))
        self._vel_avg = WindowedAverage(averaging_window_size)
        self._acceleration = Derivative(initial_value=np.array([0, 0, 0]))
        self._acc_avg = WindowedAverage(averaging_window_size)

        self._ang_velocity = Derivative(initial_value=0, value_range=[-np.pi, np.pi])
        self._ang_vel_avg = WindowedAverage(averaging_window_size)
        self._ang_acceleration = Derivative(initial_value=0)
        self._ang_acc_avg = WindowedAverage(averaging_window_size)

    def tf_handler(self, msg):
        for tf in msg.transforms:
            self.tf_update(tf)

    def tf_update(self, tf):
        """
        Update tf tree
        """
        key = tf.child_frame_id
        self._parents[key] = tf.header.frame_id
        self._last_tfs[key] = tf.transform

        if key != self._body_frame:
            return False

        # Update vel/acc on the body frame update
        time = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9

        vel = self._velocity.update(self.position, time)
        vel_avg = self._vel_avg.update(vel)
        acc = self._acceleration.update(vel_avg, time)
        self._acc_avg.update(acc)

        ang_vel = self._ang_velocity.update(self.rotation, time)
        ang_vel_avg = self._ang_vel_avg.update(ang_vel)
        ang_acc = self._ang_acceleration.update(ang_vel_avg, time)
        self._ang_acc_avg.update(ang_acc)

        for callback in self._pose_change_callbacks:
            callback(self)

        return True

    def _build_chain(self, frame):
        """
        Build a chain of related transforms
        """
        if frame not in self._parents:
            return []
        parent = self._parents[frame]
        if parent == self._root_frame:
            return []

        return [parent] + self._build_chain(parent)

    def _get_pose_from_root(self, frame):
        """
        Get the pose of a frame from the root frame
        """
        if frame not in self._parents or frame not in self._last_tfs:
            return None

        tf = self._last_tfs[frame]
        m = transform_to_matrix(tf)
        for parent in self._build_chain(frame):
            ptf = self._last_tfs[parent]
            m = np.matmul(transform_to_matrix(ptf), m)

        return np.around(m, decimals=2)

    @property
    def position(self):
        """
        Nice vector from tfs
        """
        pos = self._get_pose_from_root(self._body_frame)
        if pos is None:
            return np.array([0, 0, 0])
        return pos[0:3, 3]

    @property
    def rotation(self):
        """
        Nice number from tfs
        """
        frame = self._get_pose_from_root(self._body_frame)
        if frame is None:
            return 0
        # Ref: http://planning.cs.uiuc.edu/node103.html
        return np.arctan2(frame[1, 0], frame[0, 0])

    @property
    def velocity(self):
        """
        Nice custom vector object, from current value and frame
        """
        return VehicleVec(self._vel_avg.value, self._get_pose_from_root(self._body_frame))

    @property
    def acceleration(self):
        """
        Nice custom vector object, from current value and frame
        """
        return VehicleVec(self._acc_avg.value, self._get_pose_from_root(self._body_frame))

    @property
    def angular_velocity(self):
        """
        Simple property wrapper
        """
        return self._ang_vel_avg.value

    @property
    def angular_acceleration(self):
        """
        Simple property wrapper
        """
        return self._ang_acc_avg.value

    @property
    def wheel_angles(self):
        """
        Calculate the angles of each wheel
        """
        angles = {}

        for side, wheel_frame in self._hinge_frames.items():
            if wheel_frame not in self._last_tfs:
                continue
            tf = self._last_tfs[wheel_frame]
            angles[side] = quat_to_euler(tf.rotation)["yaw"]

        return angles
