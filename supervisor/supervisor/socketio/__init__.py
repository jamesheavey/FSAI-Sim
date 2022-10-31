from .. import socketio, ros, supervisor_state, app
# from ..setup import Setup
from .simulator import *        # noqa: F401, F403
import time
# import json

profiling = False
profile = {}
start_time = 0


@socketio.on("get_state")
def get_state(_):
    return supervisor_state.to_dict()


@supervisor_state.on_update
def update_state(state):
    socketio.emit("state_update", state.to_dict())


@socketio.on("get_accumulators")
def get_accumulators(_):
    return [
        {"name": name, "data": data}
        for name, data in supervisor_state.accumulators.get_all_data().items()
    ]


# @supervisor_state.accumulators.on_update
# def accumulator_new_item(name, action, data):
#     payload = {"name": name, "action": action, "data": data}
#     if profiling:
#         profile[name] = profile.get(name, []) + [len(json.dumps(payload))]
#     socketio.emit("accumulator_update",
#                   payload
#                   )


# @supervisor_state.dispatchers.on_update
# def dispatcher_on_update(name, data):
#     payload = {"name": name, "data": data}
#     if profiling:
#         profile[name] = profile.get(name, []) + [len(json.dumps(payload))]
#     socketio.emit("dispatcher_update",
#                   payload
#                   )


@supervisor_state.aggregator.on_update
def combined_on_update(data):
    socketio.emit("clock_update",
                  data
                  )


@socketio.on('message')
def handle_message(message):
    app.logger.info(f'received message: {message}')


@socketio.on("set_run_rules")
def handle_set_rules(data):
    return supervisor_state.run_rule_configuration.set_configuration(data)


@socketio.on("set_fuzzing")
def handle_set_fuzzing(data):
    supervisor_state.fuzz_controller.set_configuration(data)
    supervisor_state.fuzz_controller.apply_configuration()
    return supervisor_state.fuzz_controller.to_dict()


@socketio.on("publish")
def handle_publish_request(topic, data):
    if topic == "/stress/bandwidth":
        rate = data.get("time", None)
        duration = float(data.get("duration", 10))
        payload = data.get("payload", {"Item": "Hello world"})
        app.logger.info("Start stress test")
        start_t = time.time()
        while time.time() - start_t < duration:
            socketio.emit("accumulator_update",
                          {"name": "stress", "action": "append", "data": payload}
                          )
            if rate is not None:
                time.sleep(float(rate))
        socketio.emit("accumulator_update",
                      {"name": "stress", "action": "clear", "data": {}}
                      )
    else:
        global profiling, start_time, profile
        if not profiling:
            profiling = True
            start_time = time.time()
            profile = {}
            app.logger.info("Profiling started")
        else:
            profiling = False
            dt = time.time() - start_time
            app.logger.info("Profiling ended {}\n{}\n{}".format(
                dt,
                {key: len(value) / dt for key, value in profile.items()},
                {key: sum(value) / len(value) for key, value in profile.items()},
            ))
        ros.publish(topic, data)
