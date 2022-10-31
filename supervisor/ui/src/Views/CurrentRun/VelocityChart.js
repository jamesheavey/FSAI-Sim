import React, { useContext, useState, useEffect, useRef } from "react";
import { Context } from "../../App";
import Grid from "@material-ui/core/Grid";
import ElementHeader from "../../Components/ElementHeader";
import CanvasVelocityChart from "../../Components/CanvasVelocityChart";

const getVelocityData = (context) => {
  const vel = context.accumulators?.vehicle_velocity || [];
  const laps = context.accumulators?.laps || [];

  const transformVelocity = (vel, sample = 1) =>
    vel.map((v, i) => ({ y: v.x, x: i })).filter((v, i) => i % sample == 0);

  const data = [
    ...laps.map((lap) => ({
      id: `Lap ${lap.lap} velocity`,
      data: transformVelocity(lap.vehicle_velocity || []),
      color: "#aaaa",
    })),
    {
      id: "velocity",
      data: transformVelocity(vel),
      color: "#ff0000",
    },
  ];

  return data;
};

const VelocityChart = ({ width, height, ...props }) => {
  const context = useContext(Context);

  const data = getVelocityData(context);

  return (
    <>
      <ElementHeader {...props} />
      <Grid
        container
        direction="column"
        justify="flex-start"
        alignItems="center"
      >
        <CanvasVelocityChart
          height={height * 1.05}
          width={width - 30}
          data={data}
        />
      </Grid>
    </>
  );
};

export default VelocityChart;
