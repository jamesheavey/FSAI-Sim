import React, { useState, useContext, useEffect, useRef } from "react";
import clsx from "clsx";
import Title from "../../Components/Title";
import { Context } from "../../App";
import { DashboardContext } from "../../Dashboard";
import { makeStyles } from "@material-ui/core/styles";
import IconButton from "@material-ui/core/IconButton";
import Input from "@material-ui/core/Input";
import FilledInput from "@material-ui/core/FilledInput";
import OutlinedInput from "@material-ui/core/OutlinedInput";
import InputLabel from "@material-ui/core/InputLabel";
import InputAdornment from "@material-ui/core/InputAdornment";
import FormHelperText from "@material-ui/core/FormHelperText";
import FormControl from "@material-ui/core/FormControl";
import TextField from "@material-ui/core/TextField";
import Visibility from "@material-ui/icons/Visibility";
import VisibilityOff from "@material-ui/icons/VisibilityOff";
import Grid from "@material-ui/core/Grid";
import CasinoIcon from "@material-ui/icons/Casino";
import Tooltip from "@material-ui/core/Tooltip";
import Select from "@material-ui/core/Select";
import MenuItem from "@material-ui/core/MenuItem";

const generateSeed = () => Math.floor(Math.random() * 1e8);

const complexity = {
  basic: [0.8, 4],
  medium: [0.5, 1.5],
  complex: [0.2, 1],
};

const RandomMapCustomisation = ({}) => {
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);

  const mapProps = dashboardContext?.setup?.mapProps || {};
  const setMapProps = (props) =>
    dashboardContext.dispatchSetup({ mapProps: props });

  useEffect(() => {
    setMapProps({
      seed: generateSeed(),
      track_width: 3,
      size: 50,
      complexity: complexity.medium,
    });
  }, []);

  const mapAvailable =
    !!context?.state?.current_run && !!context?.state?.options;
  const currentStatus = context?.state?.current_run?.gazebo_status;

  const showLaunch = mapAvailable && currentStatus == 0;

  return (
    <>
      <Title>Track customisation</Title>
      <Grid container spacing={1}>
        <Grid item xs={12}>
          <FormControl variant="outlined" fullWidth>
            <InputLabel htmlFor="outlined-adornment-password">Seed</InputLabel>
            <OutlinedInput
              id="outlined-adornment-password"
              fullWidth
              value={mapProps.seed || 0}
              onChange={e => setMapProps({...mapProps, seed: parseInt(e.target.value)})}
              type="number"
              disabled={!showLaunch}
              endAdornment={
                <InputAdornment position="end">
                  <Tooltip title="Randomise">
                    <IconButton
                      onClick={(e) => {
                        e.preventDefault();
                        setMapProps({ ...mapProps, seed: generateSeed() });
                      }}
                      onMouseDown={(e) => e.preventDefault()}
                      edge="end"
                      disabled={!showLaunch}
                    >
                      <CasinoIcon />
                    </IconButton>
                  </Tooltip>
                </InputAdornment>
              }
              labelWidth={70}
            />
          </FormControl>
        </Grid>
        <Grid item xs={12}>
          <FormControl variant="outlined" fullWidth>
            <InputLabel id="select-size">Track size</InputLabel>
            <Select
              labelId="select-size"
              variant="outlined"
              value={mapProps.size || 30}
              disabled={!showLaunch}
              onChange={(e) =>
                setMapProps({ ...mapProps, size: e.target.value })
              }
            >
              {[
                { value: 30, label: "small" },
                { value: 50, label: "medium" },
                { value: 100, label: "large" },
              ].map((m) => (
                <MenuItem value={m.value} key={m.value}>
                  {m.label}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
        </Grid>
        <Grid item xs={12}>
          <FormControl variant="outlined" fullWidth>
            <InputLabel id="select-complexity">Track complexity</InputLabel>
            <Select
              labelId="select-complexity"
              variant="outlined"
              value={mapProps.complexity || 30}
              disabled={!showLaunch}
              onChange={(e) =>
                setMapProps({ ...mapProps, complexity: e.target.value })
              }
            >
              {[
                { value: complexity.basic, label: "basic" },
                { value: complexity.medium, label: "medium" },
                { value: complexity.complex, label: "complex" },
              ].map((m) => (
                <MenuItem value={m.value} key={m.value}>
                  {m.label}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
        </Grid>
        <Grid item xs={12}>
          <FormControl variant="outlined" fullWidth>
            <InputLabel id="select-track-width">Track width</InputLabel>
            <Select
              labelId="select-track-width"
              variant="outlined"
              value={mapProps.track_width || 3}
              disabled={!showLaunch}
              onChange={(e) =>
                setMapProps({ ...mapProps, track_width: e.target.value })
              }
            >
              {[3, 3.5, 4, 4.5, 5, 5.5, 6].map((m) => (
                <MenuItem value={m} key={m}>
                  {m}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
        </Grid>
      </Grid>
    </>
  );
};

export default RandomMapCustomisation;
