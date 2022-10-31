import React, { useContext, useState, useEffect, useRef } from "react";
import { Context } from "../../App";
import Grid from "@material-ui/core/Grid";
import ElementHeader from "../../Components/ElementHeader";
import CanvasVelocityChart from "../../Components/CanvasVelocityChart";

const getCrossTrackData = (context) => {
  const ct = context.accumulators?.rms_cross_track || [];
  const laps = context.accumulators?.laps || [];

  const transformRmsCte = (ct, sample = 1) =>
    ct.map((v, i) => ({ y: v, x: i })).filter((v, i) => i % sample == 0);

  const data = [
    ...laps.map((lap) => ({
      id: `Lap ${lap.lap} RMS cross track error`,
      data: transformRmsCte(lap.rms_cross_track || []),
      color: "#aaaa",
    })),
    {
      id: "RMS cross track error",
      data: transformRmsCte(ct),
      color: "#ff0000",
    },
  ];

  return data;
};

const CrossTrackChart = ({ width, height, ...props }) => {
  const context = useContext(Context);

//   useEffect(() => {
//     console.log("cross track", context.accumulators?.cross_track);
//   }, [context.accumulators?.cross_track])

  const data = getCrossTrackData(context);

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
          yAxisTitle="RMS CTE (m^2)"
        />
      </Grid>
    </>
  );
};

export default CrossTrackChart;