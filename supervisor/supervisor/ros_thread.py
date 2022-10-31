import threading
import time
import rclpy


class RosThread(threading.Thread):
    """
    Thread to spin the ros node
    Init node from a node_factory on init
    Pass listen and publish requests directly
    Put service requests in a queue, and let the thread call them, to avoid cross-thread issues
    """

    def __init__(self, node_factory):
        threading.Thread.__init__(self)
        self.node_factory = node_factory
        self.node = None
        self.service_queue = []
        self.stop = False
        self.setup_queue = []
        self.daemon = True

    def init(self, args):
        rclpy.init(args=args)
        self.node = self.node_factory()

        # Handle setup actions done before the node was created
        for action, data in self.setup_queue:
            if action == "listener":
                topic, callback = data
                self.set_listener_lambda(topic, callback)
            else:
                raise Exception(f"Unknown action {action}")

    def run(self):
        assert self.node, "Thread not initialised"

        while not self.stop and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=1)

            if len(self.service_queue) <= 0:
                continue

            name, callback, args, kwargs = self.service_queue.pop()
            self.node.send_request(name, callback, *args, **kwargs)

        self.node.destroy_node()
        rclpy.shutdown()

    def service_request(self, name, callback, *args, **kwargs):
        self.service_queue.append((name, callback, args, kwargs))

    def service_request_blocking(self, name, *args, **kwargs):
        results = []

        def handler(result):
            results.append(result)

        self.service_queue.append((name, handler, args, kwargs))

        while len(results) <= 0:
            time.sleep(0.1)
        return results[0]

    def set_listener_lambda(self, topic, callback):
        if self.node:
            self.node.set_listener(topic, callback)
        else:
            self.setup_queue.append(("listener", (topic, callback)))

    def listener(self, topic):
        def decorator(handler):
            self.set_listener_lambda(topic, handler)
            return handler
        return decorator

    def publish(self, topic, *args, **kwargs):
        self.node.publish(topic, *args, **kwargs)

    @property
    def log(self):
        return self.node.get_logger()

    def shutdown(self):
        self.stop = True
