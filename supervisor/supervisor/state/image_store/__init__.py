import cv2
from cv_bridge import CvBridge, CvBridgeError


class ImageStore:
    """
    Store some image published on a topic
    Only hold the last image
    This is a push-in, pull-out data structure
    """

    def __init__(self, ros, topic):
        self.last_image = None
        self.last_frame_id = 0
        self.ros = ros
        self.ros.set_listener_lambda(
            topic,
            lambda _, msg: self._set_last_image(msg)
        )
        self.bridge = CvBridge()

    def _set_last_image(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_frame_id += 1
        except CvBridgeError:
            pass

    def get_last_image_jpeg(self, scale=1):
        try:
            img = self.last_image
            if scale < 1:
                img = cv2.resize(img, (0, 0), fx=scale, fy=scale)
            _, buffer = cv2.imencode('.jpg', img)
            return buffer.tobytes()
        except Exception:
            return None
