import React, { useContext, useState, useEffect } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import { DashboardContext } from "../../Dashboard";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import Select from "@material-ui/core/Select";
import MenuItem from "@material-ui/core/MenuItem";
import FormControlLabel from "@material-ui/core/FormControlLabel";
import Checkbox from "@material-ui/core/Checkbox";

const SimulatorManager = () => {
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);
  const setupState = dashboardContext.setup || {};
  const setSetupState = dashboardContext.dispatchSetup;
  const { addError } = dashboardContext;
  const mapProps = dashboardContext?.setup?.mapProps || {};
  const mapFuzzing = dashboardContext?.setup?.mapFuzz || {};

  const selectedCar = setupState.selectedCar || "";
  const setSelectedCar = (_selectedCar) => {
    setSetupState({ selectedCar: _selectedCar });
  };

  const withGui = !!setupState.withGui;
  const setWithGui = (_withGui) => {
    setSetupState({ withGui: _withGui });
  };

  const selectedMap = setupState.selectedMap || "";
  const setSelectedMap = (_selectedMap) => {
    setSetupState({ selectedMap: _selectedMap });
  };

  const selectedDrivers = setupState.selectedDriver || [];
  const setSelectedDrivers = (drivers) => {
    setSetupState({ selectedDriver: drivers });
  };

  const [loading, setLoading] = useState(false);

  const handleLaunchMap = () => {
    setLoading(true);
    context.actions.gazebo.launchMap({map: selectedMap, params: {...mapProps, fuzzing: mapFuzzing}}, withGui, (result) => {
      setLoading(false);
      console.log(result);
      if (!result) {
        addError({ title: "Failed to launch map", severity: "error" });
      }
    });
  };

  const handleStopMap = () => {
    setLoading(true);
    context.actions.gazebo.stopMap((result) => {
      setLoading(false);
      console.log(result);
      if (!result) {
        addError({ title: "Failed to stop map", severity: "error" });
      }
    });
  };

  const handleSpawnCar = () => {
    setLoading(true);
    context.actions.gazebo.spawnCar(selectedCar.name, (result) => {
      setLoading(false);
      console.log(result);
      if (!result) {
        addError({ title: "Failed to spawn car", severity: "error" });
      }
    });
  };

  const handleLaunchMapSpawnCar = () => {
    setLoading(true);
    context.actions.gazebo.launchMapSpawnCar(
      {map: selectedMap, params: {...mapProps, fuzzing: mapFuzzing}},
      withGui,
      selectedCar.name,
      (result) => {
        setLoading(false);
        console.log(result);
        if (!result) {
          addError({
            title: "Failed to launch map and spawn car",
            severity: "error",
          });
        }
      }
    );
  };

  const mapAvailable =
    !!context?.state?.current_run && !!context?.state?.options;
  const currentStatus = context?.state?.current_run?.gazebo_status;

  const showLaunch = mapAvailable && !loading && currentStatus == 0;
  const showStop = mapAvailable && !loading && currentStatus != 0;
  const showCarSpawn = showStop && !context?.state?.current_run?.car?.name;
  const statusLookup = {
    "-1": "unknown",
    0: "Not running",
    1: "Running",
    2: "Error",
  };

  useEffect(() => {
    console.log("mapAvailable", mapAvailable && !selectedMap);
    if (mapAvailable && !selectedCar) {
      setSelectedCar(context.state.options.cars[0]);
    }
    if (mapAvailable && !selectedMap) {
      setSelectedMap(context.state.options.maps[0]);
    }
  }, [mapAvailable]);

  const drivers = context?.state?.options?.drivers?.map(d => `${d[0]}:${d[1]}`) || []

  useEffect(() => {
    console.log(selectedDrivers, drivers);
    if (selectedDrivers?.length <= 0 && drivers.length > 0) {
      setSelectedDrivers([drivers[0]]);
      console.log("Set selected drivers", [drivers[0]])
    }
  }, [context?.state?.options?.drivers]);

  useEffect(() => {
    console.log("selected", selectedDrivers, setupState)
  }, [selectedDrivers])

  // console.log(selectedMap, selectedCar)

  return (
    <>
      <Title>Simulator manager: {statusLookup[currentStatus]}</Title>
      <Grid container spacing={1} style={{ width: "100%" }}>
        <Grid item style={{ width: "100%" }}>
          <Select
            variant="outlined"
            style={{ width: "100%" }}
            value={selectedMap}
            // renderValue={(map) => map.name}
            disabled={!showLaunch}
            onChange={(e) => setSelectedMap(e.target.value)}
          >
            {context?.state?.options?.maps?.map((m) => (
              <MenuItem value={m} key={m}>
                {m}
              </MenuItem>
            ))}
          </Select>
          <FormControlLabel
            control={
              <Checkbox
                checked={withGui}
                onChange={(e) => setWithGui(e.target.checked)}
                name="checkedB"
                color="primary"
                disabled={!showLaunch}
              />
            }
            label="With GUI"
          />
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Select
            variant="outlined"
            style={{ width: "100%" }}
            value={selectedCar}
            renderValue={(car) => car.name}
            disabled={!(showLaunch || showCarSpawn)}
            onChange={(e) => setSelectedCar(e.target.value)}
          >
            {context?.state?.options?.cars?.map((m) => (
              <MenuItem value={m} key={m.name}>
                {m.name}
              </MenuItem>
            ))}
          </Select>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleLaunchMapSpawnCar}
            style={{ width: "100%" }}
            disabled={!showLaunch}
          >
            {"Launch Gazebo & spawn car"}
          </Button>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleLaunchMap}
            style={{ width: "100%" }}
            disabled={!showLaunch}
          >
            Launch Gazebo
          </Button>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleSpawnCar}
            style={{ width: "100%" }}
            disabled={!showCarSpawn}
          >
            Spawn Car
          </Button>
        </Grid>
        <hr />
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="secondary"
            onClick={handleStopMap}
            style={{ width: "100%" }}
            disabled={!showStop}
          >
            Stop Gazebo
          </Button>
        </Grid>
      </Grid>
    </>
  );
};

export default SimulatorManager;
