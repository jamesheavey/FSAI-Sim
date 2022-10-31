import React, { useContext, useState, useEffect, useRef } from "react";
import IconButton from "@material-ui/core/IconButton";
import RotateLeftIcon from "@material-ui/icons/RotateLeft";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";
import PauseIcon from "@material-ui/icons/Pause";
import { Context } from "../../App";
import Grid from "@material-ui/core/Grid";
import Title from "../../Components/Title";
import { Tooltip, Typography, Toolbar } from "@material-ui/core";
import { ResponsiveScatterPlotCanvas } from "@nivo/scatterplot";
import ElementHeader from "../../Components/ElementHeader";
import { CastSharp, Minimize } from "@material-ui/icons";
import CanvasGGDiagram from "../../Components/CanvasGGDiagram";

const getAccelerationData = (context) => {
  const g = 9.81;
  const laps = context.accumulators?.laps || [];

  const transformAcceleration = (acc, sample = 1) =>
    acc
      .map((a) => ({
        x: (a.y / g),
        y: (a.x / g),
      }))
      .filter((a, i) => i % sample == 0);

  const acc = transformAcceleration(
    context.accumulators.vehicle_acceleration || []
  );

  const data = [
    ...laps.map((lap) => ({
      id: `Lap ${lap.lap} gg`,
      data: transformAcceleration(lap.vehicle_acceleration || [], 5),
      color: "#aaaaaa",
    })),
    {
      id: "gg",
      data: acc,
      color: "#ff0000",
      scaler: true,
    },
  ];
  return data;
};

const GGDiagram = ({ width, height, ...props }) => {
  const context = useContext(Context);

  const data = getAccelerationData(context);

  return (
    <>
      <ElementHeader {...props} />
      <CanvasGGDiagram data={data} height={height * 1.05} width={width} />
    </>
  );
};

export default GGDiagram;
