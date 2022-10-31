from .. import socketio, simulator_manager, map_manager
from fs_utils.conversions import map_to_dict


@socketio.on("simulator/map_preview")
def get_map_preview(data):
    if "name" not in data:
        return {}
    return map_to_dict(map_manager.get_map(data["name"], data.get("params", {})))


@socketio.on("simulator/launch")
def launch_map(data):
    return simulator_manager.launch(data["map"], data["gui"])


@socketio.on("simulator/launch_spawn_car")
def launch_gazebo_spawn_car(data):
    return simulator_manager.launch_spawn_car(data["map"], data["gui"], data["car"])


@socketio.on("simulator/stop")
def stop_map(*_):
    return simulator_manager.stop()


@socketio.on("simulator/spawn_car")
def spawn_car(data):
    return simulator_manager.spawn_car(data["car"])


@socketio.on("simulator/reset")
def reset_gazebo(*_):
    return simulator_manager.reset()


@socketio.on("simulator/pause")
def pause_gazebo(*_):
    return simulator_manager.pause()


@socketio.on("simulator/unpause")
def unpause_gazebo(*_):
    return simulator_manager.unpause()


@socketio.on("simulator/start_driver")
def start_driver(data):
    return simulator_manager.start_driver(data)


@socketio.on("simulator/stop_driver")
def stop_driver(*_):
    return simulator_manager.stop_driver()


@socketio.on("simulator/restart_driver")
def restart_driver(*_):
    return simulator_manager.restart_driver()
