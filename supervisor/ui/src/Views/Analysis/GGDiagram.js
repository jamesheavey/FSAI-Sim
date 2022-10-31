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
import { getSelectedDataOfType, getDataOfType } from "./util";

const g = 9.81;

const processAccelerationData = (acc) => {
  return acc?.map((a) => ({
    x: a.y / g,
    y: a.x / g,
  }));
};

const GGDiagram = ({ width, height, report, selected, ...props }) => {
  const selectionTolerance = 0.005;
  const data = selected
    ? [
        {
          id: "all_gg",
          data: processAccelerationData(getDataOfType(report, "acceleration")),
          color: "#aaa",
        },
        {
          id: "gg",
          data: processAccelerationData(
            getSelectedDataOfType(report, "acceleration")
          ),
          color: "#a99",
          scaler: true,
        },
        {
          id: "selected_gg",
          data: processAccelerationData(
            getSelectedDataOfType(report, "acceleration")?.filter(
              (p) => Math.abs(p.location.ratio - selected) < selectionTolerance
            )
          ),
          color: "#f00",
        },
      ]
    : [
        {
          id: "all_gg",
          data: processAccelerationData(getDataOfType(report, "acceleration")),
          color: "#aaa",
        },
        {
          id: "gg",
          data: processAccelerationData(
            getSelectedDataOfType(report, "acceleration")
          ),
          color: "#f00",
          scaler: true,
        },
      ];

  return (
    <>
      <ElementHeader {...props} />
      <CanvasGGDiagram data={data} height={height * 1.05} width={width} />
    </>
  );
};

export default GGDiagram;
