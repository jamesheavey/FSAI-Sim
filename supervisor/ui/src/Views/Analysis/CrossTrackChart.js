import React, { useContext, useState, useEffect, useRef } from "react";
import Grid from "@material-ui/core/Grid";
import ElementHeader from "../../Components/ElementHeader";
import CanvasVelocityChart from "../../Components/CanvasVelocityChart";
import { getSelectedDataOfType, getDataOfType } from "./util";

const processRmsCteData = ({ id, data, ...props }) => {
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
      y: p.centerline?.distance,
      x: p.location.ratio,
    })),
    ...props,
  }));
};

const CrossTrackChart = ({ width, height, report, selected, setSelected, ...props }) => {
  const data = [
    ...processRmsCteData({
      id: "RMS CTE",
      data: getDataOfType(report, "pos_on_map"),
      color: "#aaa",
    }),
    ...processRmsCteData({
      id: "Lap",
      data: getSelectedDataOfType(report, "pos_on_map"),
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
          yAxisTitle="RMS CTE (m^2)"
          selected={selected}
          setSelected={setSelected}
        />
      </Grid>
    </>
  );
};

export default CrossTrackChart;
