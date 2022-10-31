import React, { useContext, useState, useEffect, useRef } from "react";
import IconButton from "@material-ui/core/IconButton";
import RotateLeftIcon from "@material-ui/icons/RotateLeft";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";
import PauseIcon from "@material-ui/icons/Pause";
import { Context } from "../../App";
import Grid from "@material-ui/core/Grid";
import Title from "../../Components/Title";
import { Tooltip, Typography, Toolbar } from "@material-ui/core";
import ElementHeader from "../../Components/ElementHeader";
import StopIcon from '@material-ui/icons/Stop';

const TimeController = (props) => {
  const context = useContext(Context);
  const running = context.state?.current_run?.running;
  const onPlay = () => {
    context.actions?.gazebo?.unpauseMap();
  };
  const onPause = () => {
    context.actions?.gazebo?.pauseMap();
  };
  const onReset = () => {
    context.actions?.gazebo?.resetMap();
  };
  const onStop = () => {
    context.actions.gazebo.stopMap();
  };
  return (
    <>
      <ElementHeader {...props} />
      <Grid
        container
        spacing={2}
        direction="row"
        justify="flex-start"
        alignItems="center"
      >
        {running ? (
          <Tooltip title="pause">
            <IconButton onClick={onPause} size="small">
              <PauseIcon />
            </IconButton>
          </Tooltip>
        ) : (
          <Tooltip title="unpause">
            <IconButton onClick={onPlay} size="small">
              <PlayArrowIcon  />
            </IconButton>
          </Tooltip>
        )}
        <Tooltip title="reset">
          <IconButton onClick={onReset} size="small">
            <RotateLeftIcon  />
          </IconButton>
        </Tooltip>
        <Tooltip title="stop">
          <IconButton onClick={onStop} size="small">
            <StopIcon  />
          </IconButton>
        </Tooltip>
        <Typography>
          Time: {Math.round((context.dispatchers.clock || 0) * 100) / 100}
        </Typography>
      </Grid>
    </>
  );
};

export default TimeController;
