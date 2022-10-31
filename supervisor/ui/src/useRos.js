import React, { useState, useEffect, useReducer } from "react";
import io from "socket.io-client";
// import _ from 'lodash';

const stateReducer = (state, action) => {
  switch (action.type) {
    case "connect":
      return { ...state, socket: action.socket };
    case "disconnect":
      return { ...state, socket: null };
    case "update":
      return { ...state, ...action.data };
    default:
      throw new Error();
  }
};

const accumulatorsReducer = (state, action) => {
  const setAction = (_state, _action) => ({
    ..._state,
    [_action.name]: _action.data,
  });
  const appendAction = (_state, _action) => ({
    ..._state,
    [_action.name]: [...(_state[_action.name] || []), _action.data], //.slice(-500),
  });
  const clearAction = (_state, _action) => ({ ..._state, [_action.name]: [] });

  const switchAction = (_state, _action) => {
    switch (_action.type) {
      case "set":
        return setAction(_state, _action);
      case "append":
        return appendAction(_state, _action);
      case "clear":
        return clearAction(_state, _action);
      case "multiple":
        return _action.actions.reduce(
          (__state, __action) =>
            switchAction(__state, {
              type: __action.action,
              ...__action,
            }),
          _state
        );
      default:
        throw new Error(`Unknown action type: ${_action.type}`);
    }
  };
  return switchAction(state, action);
};

const dispatcherReducer = (state, action) => {
  const updateAction = (_state, _action) => ({
    ..._state,
    [_action.name]: _action.data,
  });

  const switchAction = (_state, _action) => {
    // console.log("switch", _action);
    switch (_action.type) {
      case "update":
        return updateAction(_state, _action);
      case "multiple":
        return _action.actions.reduce(
          (__state, __action) =>
            switchAction(__state, {
              type: "update",
              ...__action,
            }),
          _state
        );
      default:
        throw new Error();
    }
  };

  return switchAction(state, action);
};

const profilerReducer = (_state, _action) => {
  switch (_action.type) {
    case "update":
      return {
        ..._state,
        [_action.metric]: _action.aggregation(
          _state[_action.metric],
          _action.value
        ),
      };
    case "reset":
      return _state;
    default:
      throw new Error();
  }
};

const useRos = () => {
  const [state, dispatchState] = useReducer(stateReducer, {});
  const [accumulators, dispatchAccumulators] = useReducer(
    accumulatorsReducer,
    {}
  );
  const [dispatchers, dispatchDispatchers] = useReducer(dispatcherReducer, {});
  const [metrics, dispatchMetrics] = useReducer(profilerReducer, {});

  useEffect(() => {
    console.log("creating socket");
    const _socket = io({ reconnection: true });
    console.log("created");
    console.log(_socket);

    _socket.on("connect", () => {
      console.log("connect");
      dispatchState({ type: "connect", socket: _socket });
      _socket.emit("message", { data: "I'm react" });

      _socket.emit("get_state", {}, (result) =>
        dispatchState({ type: "update", data: result })
      );

      _socket.emit("get_accumulators", {}, (result) =>
        result.map((r) => dispatchAccumulators({ type: "set", ...r }))
      );
    });

    _socket.on("reconnect", () => {
      console.log("reconnected");
    });

    _socket.on("disconnect", () => {
      dispatchState({ type: "disconnect" });
      console.log("disconnect");
    });

    _socket.on("state_update", (data) => {
      dispatchState({ type: "update", data });
    });

    _socket.on("accumulator_update", (data) => {
      dispatchAccumulators({ type: data.action, ...data });
    });

    _socket.on("dispatcher_update", (data) => {
      dispatchDispatchers({ type: "update", ...data });
    });

    _socket.on("clock_update", (data) => {
      const byActionType = data.actions.reduce(
        (obj, action) => ({
          ...obj,
          [action.actionType]: [...(obj[action.actionType] || []), action.data],
        }),
        {}
      );
      dispatchMetrics({
        type: "update",
        metric: "clock_update_time",
        value: Date.now(),
        aggregation: (obj, value) => ({
          dt: (value - (obj?.last || value) + (obj?.dt || 0) * 10) / 11,
          last: value,
        }),
      });
      dispatchDispatchers({
        type: "multiple",
        actions: byActionType.dispatcher || [],
      });
      dispatchAccumulators({
        type: "multiple",
        actions: byActionType.accumulator || [],
      });
    });

    return () => {
      _socket.disconnect();
      dispatchState({ type: "disconnect" });
      console.log("disconnected");
    };
  }, [dispatchState]);

  useEffect(() => {
    console.log("state", state);
  }, [state]);

  // useEffect(() => {
  //   console.log("metrics", metrics?.clock_update_time?.dt);
  // }, [metrics]);

  // useEffect(() => {
  //   console.log("dispatchers", dispatchers);
  // }, [dispatchers]);

  const send_service_request = (a, b) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("request", { a, b });
    }
  };

  const send_service_request_callback = (a, b, callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("request_blocking", { a, b }, callback);
    } else {
      callback({});
    }
  };

  const publish = (topic, message) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("publish", topic, message);
    }
  };

  // Gazebo actions
  const launchMap = (map, gui, callback) => {
    console.log(map, gui);
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/launch", { map, gui }, callback);
    } else {
      callback({});
    }
  };

  const launchMapSpawnCar = (map, gui, car, callback) => {
    console.log(map, gui);
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit(
        "simulator/launch_spawn_car",
        { map, gui, car },
        callback
      );
    } else {
      callback({});
    }
  };

  const stopMap = (callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/stop", {}, callback);
    } else {
      callback({});
    }
  };

  const spawnCar = (car, callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/spawn_car", { car }, callback);
    } else {
      callback({});
    }
  };

  const pauseMap = (callback) => {
    console.log("pause map");
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/pause", callback);
    } else {
      callback({});
    }
  };

  const unpauseMap = (callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/unpause", callback);
    } else {
      callback({});
    }
  };

  const resetMap = (callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/reset", callback);
    } else {
      callback({});
    }
  };

  const startDriver = (drivers, callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/start_driver", drivers, callback);
    } else {
      callback({});
    }
  };

  const stopDriver = (callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/stop_driver", callback);
    } else {
      callback({});
    }
  };

  const restartDriver = (callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/restart_driver", callback);
    } else {
      callback({});
    }
  };

  const getMapPreview = (name, params, callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("simulator/map_preview", { name, params }, callback);
    } else {
      callback({});
    }
  };

  const setRunRules = (rules, callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("set_run_rules", rules, callback);
    } else {
      callback({});
    }
  };

  const clearEvents = () => {
    dispatchDispatchers({ type: "update", name: "events", data: null });
  };

  const setFuzzing = (fuzzing, callback) => {
    if (state?.socket && state?.socket?.connected) {
      state.socket.emit("set_fuzzing", fuzzing, callback);
    } else {
      callback({});
    }
  };

  return {
    state,
    accumulators,
    dispatchers,
    actions: {
      gazebo: {
        launchMap,
        launchMapSpawnCar,
        stopMap,
        spawnCar,
        pauseMap,
        unpauseMap,
        resetMap,
        startDriver,
        stopDriver,
        restartDriver,
        getMapPreview,
      },
      send_service_request,
      send_service_request_callback,
      setRunRules,
      clearEvents,
      setFuzzing,
      publish,
    },
    metrics,
  };
};

export default useRos;
