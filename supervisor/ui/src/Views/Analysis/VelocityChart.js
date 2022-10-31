import React, { useContext, useState, useEffect, useRef } from "react";
import Grid from "@material-ui/core/Grid";
import ElementHeader from "../../Components/ElementHeader";
import CanvasVelocityChart from "../../Components/CanvasVelocityChart";
import { getSelectedDataOfType, getDataOfType } from "./util";

const processVelocityData = ({ id, data, ...props }) => {
  const dataByLap = data?.reduce(
    (obj, d) => ({
      ...obj,
      [d.location.lap]: [...(obj[d.location.lap] || []), d],
    }),
    {}
  );
  return Object.keys(dataByLap).map((lap) => ({
    id: `${id} ${lap}`,
    data: dataByLap[lap]?.map((p, i) => ({
      y: p.x,
      x: p.location.ratio,
    })),
    ...props,
  }));
};

const VelocityChart = ({ width, height, report, selected, setSelected, ...props }) => {
  const data = [
    ...processVelocityData({
      id: "velocity",
      data: getDataOfType(report, "velocity"),
      color: "#aaa",
    }),
    ...processVelocityData({
      id: "Lap",
      data: getSelectedDataOfType(report, "velocity"),
      color: "#f00",
      selectable: true
    }),
  ];

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
          xAxisTitle="location"
          selected={selected}
          setSelected={setSelected}
        />
      </Grid>
    </>
  );
};

export default VelocityChart;
