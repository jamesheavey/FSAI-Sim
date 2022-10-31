from rclpy.node import Node
import fnmatch
import time
from .topic import Publisher, Topic
from .service import Service, ServiceCallHandler


class SubscriptionHandler:
    """
    Class to capture topic on init and route messages
    """

    def __init__(self, topic, node):
        self._topic = topic
        self._node = node

    def __call__(self, msg):
        self._node.subscription_handler(self._topic, msg)


class RosNode(Node):
    """
    Node that connects ROS to the Supervisor
    Subscribe to an externally specified topics and services
    """
    def __init__(self, topics=[], services=[]):
        super().__init__('supervisor_ros_node')

        # Create services
        self._own_clients = {
            service.name: ServiceCallHandler(
                self.create_client(service.type, service.name),
                service
            )
            for service in services
        }

        # Wait for services
        while (ready_count := sum(service.ready for service in self._own_clients.values())) < len(self._own_clients):
            self.get_logger().info('Waiting for services. {}/{} available...'.format(
                ready_count, len(self._own_clients)
            ))
            time.sleep(1)
        self.get_logger().info("Services ready")

        # Create subscribers and publishers to all topics
        self._own_subscriptions = {
            topic.name: self.create_subscription(
                topic.type,
                topic.name,
                SubscriptionHandler(topic.name, self),
                topic.qos
            ) for topic in topics}

        self._own_publishers = {
            topic.name: Publisher(
                self.create_publisher(
                    topic.type, topic.name, topic.qos
                ),
                topic)
            for topic in topics}

        self.listeners = []

    def subscription_handler(self, topic, msg):
        """
        Handle all subscription messages and send to all listeners that subsribed to that topic
        """
        for listener_topic, listener_callback in self.listeners:
            if fnmatch.fnmatch(topic, listener_topic):
                listener_callback(topic, msg)

    def set_listener(self, topic, callback):
        self.listeners.append((topic, callback))

    def publish(self, topic, *args, **kwargs):
        publisher = self._own_publishers.get(topic, None)
        if not publisher:
            raise Exception(f"No publisher found for topic: {topic}")

        publisher.publish(*args, **kwargs)

    def send_request(self, name, callback, *args, **kwargs):
        def handler(future):
            callback(future.result())

        client = self._own_clients.get(name, None)
        if not client:
            raise Exception(f"No client found for service: {name}")

        future = client.call_async(*args, **kwargs)
        future.add_done_callback(handler)


def client_factory(topics, services):
    # Enforce argument types
    assert all(isinstance(topic, Topic) for topic in topics)
    assert all(isinstance(service, Service) for service in services)

    return RosNode(topics=topics, services=services)
