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
import ElementHeader from "../../Components/ElementHeader";
import TableChartIcon from "@material-ui/icons/TableChart";
import DoneIcon from "@material-ui/icons/Done";
import InsertDriveFileIcon from "@material-ui/icons/InsertDriveFile";
import Table from "@material-ui/core/Table";
import TableBody from "@material-ui/core/TableBody";
import TableCell from "@material-ui/core/TableCell";
import TableContainer from "@material-ui/core/TableContainer";
import TableHead from "@material-ui/core/TableHead";
import TableRow from "@material-ui/core/TableRow";

const DriverSelector = ({
  title = "Driver Selector",
  actions = [],
  height = 500,
}) => {
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);
  const setupState = dashboardContext.setup || {};
  const setSetupState = dashboardContext.dispatchSetup;
  const { addError } = dashboardContext;

  const selectedDrivers = setupState.selectedDriver || [];
  const setSelectedDrivers = (drivers) => {
    setSetupState({ selectedDriver: drivers });
  };

  const showTable = setupState.driverShowTable || false;
  const setShowTable = (show) => {
    setSetupState({ driverShowTable: show });
  };

  const [loading, setLoading] = useState(false);

  const mapAvailable =
    !!context?.state?.current_run && !!context?.state?.options;
  const currentStatus = context?.state?.current_run?.gazebo_status;

  const showStop = mapAvailable && !loading && currentStatus != 0;
  const showStartDriver =
    showStop &&
    !!context?.state?.current_run?.car?.name &&
    !context?.state?.current_run?.driver;
  const showStopDriver = showStop && !!context?.state?.current_run?.driver;

  const drivers =
    context?.state?.options?.drivers?.map((d) => `${d[0]}:${d[1]}`) || [];
  const fullDrivers = drivers.filter((d) => d.includes("FULL")).sort();
  const moduleDrivers = drivers.filter((d) => !d.includes("FULL")).sort();

  useEffect(() => {
    // console.log(selectedDrivers, drivers);
    if (selectedDrivers?.length <= 0 && drivers.length > 0) {
      setSelectedDrivers(showTable ? [] : [fullDrivers[0]]);
      // console.log("Set selected drivers", [drivers[0]]);
    }
  }, [context?.state?.options?.drivers]);

  //   useEffect(() => {
  //     console.log("selected", selectedDrivers, setupState);
  //   }, [selectedDrivers]);

  const handleDriverLaunchFileChange = (e) => {
    const driver = e.target.value;
    setSelectedDrivers([driver]);
  };

  const handleStartDriver = () => {
    setLoading(true);
    console.log(selectedDrivers);
    const drivers = selectedDrivers.map((d) => {
      const [pkg, launch_file, ..._] = d.split(":");
      return { package: pkg, launch_file };
    });
    context.actions.gazebo.startDriver(drivers, (result) => {
      console.log(result);
      setLoading(false);
      if (!result) {
        addError({ title: "Failed to start driver", severity: "error" });
      }
    });
  };

  const handleStopDriver = () => {
    setLoading(true);
    context.actions.gazebo.stopDriver((result) => {
      console.log(result);
      setLoading(false);
      if (!result) {
        addError({ title: "Failed to stop driver", severity: "error" });
      }
    });
  };

  const handleRestartDriver = () => {
    setLoading(true);
    context.actions.gazebo.restartDriver((result) => {
      //   console.log(result);
      setLoading(false);
      if (!result) {
        addError({ title: "Failed to stop restart driver", severity: "error" });
      }
    });
  };

  const toggleShowTable = () => {
    if (showTable) {
      setSelectedDrivers([fullDrivers[0]]);
    } else {
      setSelectedDrivers([]);
    }
    setShowTable(!showTable);
  };

  return (
    <>
      <ElementHeader
        title={title}
        actions={[
          showTable
            ? {
                title: "Select single",
                icon: <InsertDriveFileIcon fontSize="small" />,
                handler: toggleShowTable,
              }
            : {
                title: "Select multiple",
                icon: <TableChartIcon fontSize="small" />,
                handler: toggleShowTable,
              },
          ...actions,
        ]}
      />
      <Grid
        container
        spacing={1}
        style={{ width: "100%", height: "100%" }}
        direction="column"
      >
        <Grid item style={{ width: "100%" }}>
          {showTable ? (
            <TableContainer style={{ maxHeight: height - 180 }}>
              <Table size="small" stickyHeader>
                <TableHead>
                  <TableRow key={0}>
                    <TableCell component="th" scope="row">
                      Driver
                    </TableCell>
                    <TableCell align="right">Enabled</TableCell>
                  </TableRow>
                </TableHead>
                <TableBody>
                  {moduleDrivers.map((driver) => (
                    <TableRow key={driver}>
                      <TableCell component="th" scope="row">
                        {driver}
                      </TableCell>
                      <TableCell align="right">
                        <Checkbox
                          checked={selectedDrivers.includes(driver)}
                          disabled={!showStartDriver}
                          color="primary"
                          onChange={(e) => {
                            if (e.target.checked) {
                              setSelectedDrivers([...selectedDrivers, driver]);
                            } else {
                              setSelectedDrivers(
                                selectedDrivers.filter((dr) => dr !== driver)
                              );
                            }
                          }}
                        />
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            </TableContainer>
          ) : (
            <Select
              variant="outlined"
              style={{ width: "100%" }}
              value={selectedDrivers[0] || ""}
              disabled={!showStartDriver}
              onChange={handleDriverLaunchFileChange}
            >
              {fullDrivers.map((m) => (
                <MenuItem value={m} key={m}>
                  {m}
                </MenuItem>
              ))}
            </Select>
          )}
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleStartDriver}
            style={{ width: "100%" }}
            disabled={!showStartDriver || !selectedDrivers?.length}
          >
            Start driver
          </Button>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleStopDriver}
            style={{ width: "100%" }}
            disabled={!showStopDriver}
          >
            Stop driver
          </Button>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleRestartDriver}
            style={{ width: "100%" }}
            disabled={!showStopDriver}
          >
            Restart driver
          </Button>
        </Grid>
      </Grid>
    </>
  );
};

export default DriverSelector;
