import React, { useContext, useState, useEffect, useRef, useMemo } from "react";
import ElementHeader from "../../Components/ElementHeader";
import Slider from "@material-ui/core/Slider";
import IconButton from "@material-ui/core/IconButton";
import SkipNextIcon from "@material-ui/icons/SkipNext";
import SkipPreviousIcon from "@material-ui/icons/SkipPrevious";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";
import StopIcon from "@material-ui/icons/Stop";
import Grid from "@material-ui/core/Grid";
import Typography from "@material-ui/core/Typography";

const findNearestBinary = (value, marks) => {
  if (marks.length <= 0) {
    return 0;
  }
  if (marks.length == 1) {
    return marks[0];
  }
  let index = Math.floor(marks.length / 2);
  if (value >= marks[index]) {
    return findNearestBinary(value, marks.slice(index));
  } else {
    return findNearestBinary(value, marks.slice(0, index));
  }
};

const TimeSelector = ({ report, time, setTime, playing, setPlaying, ...props }) => {
  const disabled = !report;
  const min = report?.snapshots[0]?.time;
  const max = report?.snapshots?.slice(-1)[0]?.time;
  const marks = report?.snapshots?.map((snapshot) => snapshot.time);
  const playbackSpeed = 2;
  const avgDelta =
    marks?.length > 1
      ? Math.round(
          ((marks
            .map((v, i) => (i === 0 ? 0 : v - marks[i - 1]))
            .reduce((a, b) => a + b, 0) /
            (marks.length - 1)) *
            1000) /
            playbackSpeed
        )
      : 30;

  const handleChange = (event, newValue) => {
    const nearest = newValue.map((v) => findNearestBinary(v, marks));
    setTime(nearest);
    // console.log(newValue, nearest, marks);
  };

  const handleIncrement = () => {
    const index = marks?.indexOf(time[1]);
    const startIndex = marks?.indexOf(time[0]);
    if (index === undefined || startIndex === undefined) {
      return;
    }
    const next =
      index + 1 >= marks.length ? marks[startIndex] : marks[index + 1];
    setTime([time[0], next]);
  };

  const handleDecrement = () => {
    const index = marks?.indexOf(time[1]);
    if (!index) {
      return;
    }
    const prev = index - 1 < 0 ? marks.slice(-1)[0] : marks[index - 1];
    setTime([time[0], prev]);
  };

  const togglePlaying = () => {
    setPlaying(!playing);
  };

  useEffect(() => {
    if (time === null) {
      setPlaying(false);
    }
  }, [time]);

  useEffect(() => {
    if (playing) {
      const timeout = setTimeout(() => {
        handleIncrement();
      }, avgDelta);
      return () => clearTimeout(timeout);
    }
  }, [time, playing, avgDelta]);

  return (
    <Grid
      container
      direction="row"
      justify="center"
      alignItems="flex-start"
      spacing={1}
    >
      <Grid item>
        <IconButton
          size="small"
          disabled={disabled || time[1] === marks[0] || time[1] === time[0]}
          onClick={handleDecrement}
        >
          <SkipPreviousIcon />
        </IconButton>
      </Grid>
      <Grid item xs>
        <Slider
          value={time}
          onChange={handleChange}
          disabled={disabled || playing}
          min={min}
          max={max * 1.001}
          step={0.1}
        />
      </Grid>
      <Grid item style={{ marginTop: 4, minWidth: 60, textAlign: "right" }}>
        <Typography>{((time || [])[1] || 0).toFixed(2)}</Typography>
      </Grid>
      <Grid item>
        <IconButton size="small" disabled={disabled} onClick={togglePlaying}>
          {playing ? <StopIcon /> : <PlayArrowIcon />}
        </IconButton>
      </Grid>
      <Grid item>
        <IconButton
          size="small"
          disabled={disabled || time[1] === marks?.slice(-1)[0]}
          onClick={handleIncrement}
        >
          <SkipNextIcon />
        </IconButton>
      </Grid>
    </Grid>
  );
};

export default TimeSelector;
