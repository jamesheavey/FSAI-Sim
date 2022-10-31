def get_parameters(ros, keys):
    return {key: value for key in keys if (value := _declare_and_get(ros, key)) is not None}


def _declare_and_get(ros, key):
    ros.declare_parameter(key)
    return ros.get_parameter(key)._value
