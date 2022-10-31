class Service:
    """ Class to encapsulate service description
    """

    def __init__(self, name, service_type, message_factory):
        self.name = name
        self.type = service_type
        self.message_factory = message_factory


class ServiceCallHandler:
    """ Class to encapsulate calling a service through a message factory
    """

    def __init__(self, client, service):
        self._client = client
        self._message_factory = service.message_factory

    def call_async(self, *args, **kwargs):
        return self._client.call_async(self._message_factory(*args, **kwargs))

    @property
    def ready(self):
        return self._client.service_is_ready()
