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
import Table from "@material-ui/core/Table";
import TableBody from "@material-ui/core/TableBody";
import TableCell from "@material-ui/core/TableCell";
import TableContainer from "@material-ui/core/TableContainer";
import TableHead from "@material-ui/core/TableHead";
import TableRow from "@material-ui/core/TableRow";
import Box from "@material-ui/core/Box";
import FormGroup from "@material-ui/core/FormGroup";
import InputLabel from "@material-ui/core/InputLabel";
import FormHelperText from "@material-ui/core/FormHelperText";
import FormControl from "@material-ui/core/FormControl";
import IconButton from "@material-ui/core/IconButton";
import AddCircleIcon from "@material-ui/icons/AddCircle";
import Tooltip from "@material-ui/core/Tooltip";
import DeleteIcon from "@material-ui/icons/Delete";

const areSubConfigsEqual = (c1, c2) =>
  c1 &&
  c2 &&
  Object.keys(c1).every((k) =>
    Object.keys(c1[k]).every(
      (l) =>
        c1[k] &&
        c2[k] &&
        (typeof c1[k][l] === "object"
          ? Object.keys(c1[k][l]).every(
              (m) => c1[k][l] && c2[k][l] && c1[k][l][m] === c2[k][l][m]
            )
          : c1[k][l] === c2[k][l])
    )
  );

const areConfigsEqual = (r1, r2) =>
  r1 &&
  r2 &&
  areSubConfigsEqual(r1.initial, r2.initial) &&
  r1.actions.length === r2.actions.length &&
  r1?.actions.every(
    (a, i) =>
      a.time === r2.actions[i].time &&
      areSubConfigsEqual(a.config, r2.actions[i].config)
  );

const Dropdown = ({ options, value, onChange, title, disabled }) => {
  return (
    <FormControl variant="outlined" style={{ width: "100%" }}>
      <InputLabel id="dropdown">{title}</InputLabel>
      <Select
        labelId="dropdown"
        value={Object.keys(options).reduce((prev, curr) =>
          Math.abs(curr - value) < Math.abs(prev - value) ? curr : prev
        )}
        onChange={(e) => onChange(options[e.target.value])}
        label="Gaussian noise"
        disabled={disabled}
      >
        {Object.keys(options)
          .sort((a, b) => a - b)
          .map((k) => (
            <MenuItem value={k} key={k}>
              {options[k].label}
            </MenuItem>
          ))}
      </Select>
    </FormControl>
  );
};

const CameraConfig = ({ config, setConfig }) => {
  const gaussianOptions = {
    0: { label: "disabled", config: { enabled: false, variance: 0 } },
    1e-6: { label: "1e-6", config: { enabled: true, variance: 1e-6 } },
    2e-6: { label: "2e-6", config: { enabled: true, variance: 2e-6 } },
    4e-6: { label: "4e-6", config: { enabled: true, variance: 4e-6 } },
    8e-6: { label: "8e-6", config: { enabled: true, variance: 8e-6 } },
    1e-5: { label: "1e-5", config: { enabled: true, variance: 1e-5 } },
    2e-5: { label: "2e-5", config: { enabled: true, variance: 2e-5 } },
    4e-5: { label: "4e-5", config: { enabled: true, variance: 4e-5 } },
    8e-5: { label: "8e-5", config: { enabled: true, variance: 8e-5 } },
    1e-4: { label: "1e-4", config: { enabled: true, variance: 1e-4 } },
  };
  const saltAndPapperOptions = {
    0: { label: "disabled", config: { enabled: false, quantity: 0 } },
    1e-4: { label: "1e-4", config: { enabled: true, quantity: 1e-4 } },
    2e-4: { label: "2e-4", config: { enabled: true, quantity: 2e-4 } },
    4e-4: { label: "4e-4", config: { enabled: true, quantity: 4e-4 } },
    8e-4: { label: "8e-4", config: { enabled: true, quantity: 8e-4 } },
    1e-4: { label: "1e-4", config: { enabled: true, quantity: 1e-4 } },
    2e-3: { label: "2e-3", config: { enabled: true, quantity: 2e-3 } },
    4e-3: { label: "4e-3", config: { enabled: true, quantity: 4e-3 } },
    8e-3: { label: "8e-3", config: { enabled: true, quantity: 8e-3 } },
    1e-2: { label: "1e-2", config: { enabled: true, quantity: 1e-2 } },
  };

  return (
    <Grid container spacing={1}>
      <Grid item xs={4}>
        <FormControlLabel
          control={
            <Checkbox
              checked={config?.camera_enabled}
              onChange={(e) =>
                setConfig({ ...config, camera_enabled: e.target.checked })
              }
              color="primary"
            />
          }
          label="Camera enabled"
          labelPlacement="start"
        />
      </Grid>
      <Grid item xs={4}>
        <Dropdown
          options={saltAndPapperOptions}
          value={config?.snp_config?.enabled ? config?.snp_config?.quantity : 0}
          onChange={(option) =>
            setConfig({
              ...config,
              snp_config: { ...option.config, salt_pepper_ratio: 0.5 },
            })
          }
          title={"Salt & Pepper noise"}
          disabled={!config?.camera_enabled}
        />
      </Grid>
      <Grid item xs={4}>
        <Dropdown
          options={gaussianOptions}
          value={
            config?.gauss_config?.enabled ? config?.gauss_config?.variance : 0
          }
          onChange={(option) =>
            setConfig({ ...config, gauss_config: option.config })
          }
          title={"Gaussian noise"}
          disabled={!config?.camera_enabled}
        />
      </Grid>
    </Grid>
  );
};

const LidarConfig = ({ config, setConfig }) => {
  const gaussianOptions = {
    0: { label: "disabled", config: { enabled: false, variance: 0 } },
    1e-2: { label: "1e-2", config: { enabled: true, variance: 1e-2 } },
    2e-2: { label: "2e-2", config: { enabled: true, variance: 2e-2 } },
    4e-2: { label: "4e-2", config: { enabled: true, variance: 4e-2 } },
    8e-2: { label: "8e-2", config: { enabled: true, variance: 8e-2 } },
    1e-1: { label: "1e-1", config: { enabled: true, variance: 1e-1 } },
    2e-1: { label: "2e-1", config: { enabled: true, variance: 2e-1 } },
    4e-1: { label: "4e-1", config: { enabled: true, variance: 4e-1 } },
    8e-1: { label: "8e-1", config: { enabled: true, variance: 8e-1 } },
    1: { label: "1e-0", config: { enabled: true, variance: 1 } },
  };
  const saltAndPapperOptions = {
    0: { label: "disabled", config: { enabled: false, quantity: 0 } },
    1e-3: { label: "1e-3", config: { enabled: true, quantity: 1e-3 } },
    2e-3: { label: "2e-3", config: { enabled: true, quantity: 2e-3 } },
    4e-3: { label: "4e-3", config: { enabled: true, quantity: 4e-3 } },
    8e-3: { label: "8e-3", config: { enabled: true, quantity: 8e-3 } },
    1e-2: { label: "1e-2", config: { enabled: true, quantity: 1e-2 } },
    2e-2: { label: "2e-2", config: { enabled: true, quantity: 2e-2 } },
    4e-2: { label: "4e-2", config: { enabled: true, quantity: 4e-2 } },
    8e-2: { label: "8e-2", config: { enabled: true, quantity: 8e-2 } },
    1e-1: { label: "1e-1", config: { enabled: true, quantity: 1e-1 } },
  };

  return (
    <Grid container spacing={1}>
      <Grid item xs={4}>
        <FormControlLabel
          control={
            <Checkbox
              checked={config?.lidar_enabled}
              onChange={(e) =>
                setConfig({ ...config, lidar_enabled: e.target.checked })
              }
              color="primary"
            />
          }
          label="Lidar enabled"
          labelPlacement="start"
        />
      </Grid>
      <Grid item xs={4}>
        <Dropdown
          options={saltAndPapperOptions}
          value={config?.snp_config?.enabled ? config?.snp_config?.quantity : 0}
          onChange={(option) =>
            setConfig({
              ...config,
              snp_config: { ...option.config, salt_pepper_ratio: 0.5 },
            })
          }
          title={"Salt & Pepper noise"}
          disabled={!config?.lidar_enabled}
        />
      </Grid>
      <Grid item xs={4}>
        <Dropdown
          options={gaussianOptions}
          value={
            config?.gauss_config?.enabled ? config?.gauss_config?.variance : 0
          }
          onChange={(option) =>
            setConfig({ ...config, gauss_config: option.config })
          }
          title={"Gaussian noise"}
          disabled={!config?.lidar_enabled}
        />
      </Grid>
    </Grid>
  );
};

const OdomConfig = ({ config, setConfig }) => {
  const spacialOptions = {
    0: { label: "0", value: 0 },
    1e-1: { label: "1e-1", value: 1e-1 },
    2e-1: { label: "2e-1", value: 2e-1 },
    3e-1: { label: "3e-1", value: 3e-1 },
  };

  const rotOptions = {
    0: { label: "0", value: 0 },
    1e-2: { label: "1e-2", value: 1e-2 },
    2e-2: { label: "2e-2", value: 2e-2 },
    4e-2: { label: "4e-2", value: 4e-2 },
    8e-2: { label: "8e-2", value: 8e-2 },
    1e-1: { label: "1e-1", value: 1e-1 },
  };

  return (
    <Grid container spacing={1}>
      <Grid item xs={4}>
        <FormControlLabel
          control={
            <Checkbox
              checked={config?.odom_enabled}
              onChange={(e) =>
                setConfig({ ...config, odom_enabled: e.target.checked })
              }
              color="primary"
            />
          }
          label="Odom enabled"
          labelPlacement="start"
        />
      </Grid>
      <Grid item xs={4}>
        <Dropdown
          options={spacialOptions}
          value={config?.pos_covariance}
          onChange={(option) =>
            setConfig({
              ...config,
              pos_covariance: option.value,
            })
          }
          title={"XY Variance"}
          disabled={!config?.odom_enabled}
        />
      </Grid>
      <Grid item xs={4}>
        <Dropdown
          options={rotOptions}
          value={config?.rot_covariance}
          onChange={(option) =>
            setConfig({
              ...config,
              rot_covariance: option.value,
            })
          }
          title={"Angle Variance"}
          disabled={!config?.odom_enabled}
        />
      </Grid>
    </Grid>
  );
};

const FuzzConfig = ({ config, setConfig, time, setTime, onDelete }) => {
  return (
    <TableContainer>
      <Table>
        <TableHead>
          <TableRow key={0}>
            <TableCell component="th" scope="row" align="right">
              {time === undefined ? "Initial" : "T:"}
            </TableCell>
            <TableCell style={{ width: "100%" }}>
              {time === undefined ? null : (
                <>
                  <TextField
                    id="outlined-number"
                    label="Time"
                    type="number"
                    size="small"
                    InputLabelProps={{
                      shrink: true,
                    }}
                    variant="outlined"
                    value={time}
                    onChange={(e) => setTime(e.target.value)}
                    InputProps={{ inputProps: { min: 1 } }}
                  />
                  <Tooltip title={"Delete action"}>
                    <IconButton onClick={onDelete}>
                      <DeleteIcon />
                    </IconButton>
                  </Tooltip>
                </>
              )}
            </TableCell>
            <TableCell align="right">Fuzzing</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {Object.keys(config).map((key) => (
            <>
              <TableRow key={key}>
                <TableCell component="th" scope="row">
                  {key}
                </TableCell>
                <TableCell style={{ padding: 0 }}>
                  {config[key]?.fuzzing_enabled && (
                    <Box>
                      {Object.keys(config[key]).some(
                        (k) => k === "camera_enabled"
                      ) ? (
                        <CameraConfig
                          config={config[key]}
                          setConfig={(conf) =>
                            setConfig({ ...config, [key]: conf })
                          }
                        />
                      ) : Object.keys(config[key]).some(
                          (k) => k === "lidar_enabled"
                        ) ? (
                        <LidarConfig
                          config={config[key]}
                          setConfig={(conf) =>
                            setConfig({ ...config, [key]: conf })
                          }
                        />
                      ) : Object.keys(config[key]).some(
                          (k) => k === "odom_enabled"
                        ) ? (
                        <OdomConfig
                          config={config[key]}
                          setConfig={(conf) =>
                            setConfig({ ...config, [key]: conf })
                          }
                        />
                      ) : null}
                    </Box>
                  )}
                </TableCell>
                <TableCell align="right">
                  <Checkbox
                    checked={config[key].fuzzing_enabled}
                    color="primary"
                    onChange={(e) =>
                      setConfig({
                        ...config,
                        [key]: {
                          ...config[key],
                          fuzzing_enabled: e.target.checked,
                        },
                      })
                    }
                  />
                </TableCell>
              </TableRow>
            </>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );
};

const FuzzConfiguration = () => {
  const context = useContext(Context);
  const [config, setConfig] = useState(null);
  const [originalConfig, setOriginalConfig] = useState(null);

  useEffect(() => {
    // console.log(context?.state?.fuzz_controller);
    if (
      (!config &&
        context?.state?.fuzz_controller) ||
      !areConfigsEqual(context?.state?.fuzz_controller, originalConfig)
    ) {
      setConfig(context?.state?.fuzz_controller);
      setOriginalConfig(context?.state?.fuzz_controller);
    }
  }, [context?.state?.fuzz_controller]);

  useEffect(() => {
    console.log("fuzzing:", context?.state?.fuzz_controller);
  }, [context?.state?.fuzz_controller]);

  useEffect(() => {
    console.log(
      config,
      originalConfig,
      areConfigsEqual(config, originalConfig)
    );
  }, [config, originalConfig]);

  const handleSave = () => {
    const orderedConf = {
      ...config,
      actions: config.actions
        .sort((a, b) => a.time - b.time)
        .map((a) => ({ ...a, time: parseFloat(a.time) || 0 })),
    };
    context?.actions?.setFuzzing(orderedConf, (res) => {
      // console.log(res);
      setOriginalConfig(orderedConf);
    });
  };

  const hasChanges = !areConfigsEqual(originalConfig, config);

  return (
    <>
      <Title>Sensor fuzzing</Title>
      <Grid container spacing={1} style={{ width: "100%" }}>
        <TableContainer>
          <Table size="small">
            <TableRow magin={0}>
              <TableCell style={{ padding: 0 }}>
                {config?.initial && (
                  <FuzzConfig
                    config={config?.initial}
                    setConfig={(_config) =>
                      setConfig({ ...config, initial: _config })
                    }
                  />
                )}
              </TableCell>
            </TableRow>
            {config?.actions
              // ?.sort((a, b) => a.time - b.time)
              ?.map((action, i) => (
                <TableRow magin={0} key={i}>
                  <TableCell style={{ padding: 0 }}>
                    {config?.initial && (
                      <FuzzConfig
                        config={action?.config}
                        setConfig={(_config) =>
                          setConfig({
                            ...config,
                            actions: config.actions.map((a, _i) =>
                              i === _i
                                ? { ...config.actions[i], config: _config }
                                : a
                            ),
                          })
                        }
                        time={action.time}
                        setTime={(time) =>
                          setConfig({
                            ...config,
                            actions: config.actions.map((a, _i) =>
                              i === _i ? { ...config.actions[i], time } : a
                            ),
                          })
                        }
                        onDelete={() =>
                          setConfig({
                            ...config,
                            actions: config.actions.filter((a, _i) => _i !== i),
                          })
                        }
                      />
                    )}
                  </TableCell>
                </TableRow>
              ))}
            <TableRow>
              <TableCell align="center">
                <Tooltip title={"Add action"}>
                  <IconButton
                    onClick={() =>
                      setConfig({
                        ...config,
                        actions: [
                          ...config.actions,
                          {
                            time:
                              config.actions.length <= 0
                                ? 10
                                : parseFloat(
                                    config.actions
                                      .sort((a, b) => a.time - b.time)
                                      .slice(-1)[0].time
                                  ) + 10,
                            config: {
                              ...(config.actions.length <= 0
                                ? config.initial
                                : config.actions
                                    .sort((a, b) => a.time - b.time)
                                    .slice(-1)[0].config),
                            },
                          },
                        ],
                      })
                    }
                  >
                    <AddCircleIcon />
                  </IconButton>
                </Tooltip>
              </TableCell>
            </TableRow>
          </Table>
        </TableContainer>
        <hr />
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleSave}
            style={{ width: "100%" }}
            disabled={!hasChanges}
          >
            Apply
          </Button>
        </Grid>
      </Grid>
    </>
  );
};

export default FuzzConfiguration;
