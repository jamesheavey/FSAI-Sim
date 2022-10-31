import React, { useContext, useState, useEffect, useRef } from "react";
import Select from "@material-ui/core/Select";
import MenuItem from "@material-ui/core/MenuItem";
import { Context } from "../../App";
import ElementHeader from "../../Components/ElementHeader";
import Table from "@material-ui/core/Table";
import TableBody from "@material-ui/core/TableBody";
import TableCell from "@material-ui/core/TableCell";
import TableContainer from "@material-ui/core/TableContainer";
import TableHead from "@material-ui/core/TableHead";
import TableRow from "@material-ui/core/TableRow";
import GetAppIcon from "@material-ui/icons/GetApp";
import { downloadFile } from "../../utils";

const LapTimes = ({ title, actions }) => {
  const context = useContext(Context);

  const laps = context.accumulators?.laps || [];
  const totalLapTime = laps?.reduce((s, l) => s + l.lap_time, 0);
  const lastTime = (context?.dispatchers?.clock || 0) - totalLapTime;
  const penalties = context.accumulators?.penalty || [];
  const penaltiesPerLap = penalties.reduce(
    (obj, p) => ({ ...obj, [p.lap]: (obj[p.lap] || 0) + p.penalty_time }),
    {}
  );
  const currentLap = Math.max(...laps.map((l) => l.lap + 1), 0);

  const formatTime = (time) =>
    `${Math.floor(time / 60)
      .toString()
      .padStart(2, "0")}:${(time % 60).toFixed(2).padStart(5, "0")}`;

  const handleDownload = () => {
    if (!laps) {
      return;
    }
    const body = [...laps, {lap: lastLap, lap_time: currentLap}]
      .map(
        (lap) =>
          `Lap ${lap.lap + 1}: ${formatTime(lap.lap_time)}${
            penaltiesPerLap[lap.lap]
              ? `; +${penaltiesPerLap[lap.lap].toFixed(2)}s`
              : ""
          }`
      )
      .join("\n");
    downloadFile(body, `lap_times_${Math.round(Date.now() / 1000)}.txt`);
  };

  return (
    <>
      <ElementHeader
        title={title}
        actions={[
          {
            title: "Download times",
            icon: <GetAppIcon fontSize="small" />,
            handler: handleDownload,
          },
          ...(actions || []),
        ]}
      />
      <TableContainer>
        <Table size="small">
          <TableBody>
            {laps.map((lap) => (
              <TableRow key={lap.lap}>
                <TableCell component="th" scope="row">
                  {`Lap ${lap.lap + 1}`}
                </TableCell>
                <TableCell align="right">{formatTime(lap.lap_time)}</TableCell>
                {penalties.length > 0 ? (
                  <TableCell style={{ color: "red", fontWeight: "bold" }}>
                    {penaltiesPerLap[lap.lap]
                      ? `+${penaltiesPerLap[lap.lap].toFixed(2)}s`
                      : ""}
                  </TableCell>
                ) : null}
              </TableRow>
            ))}
            {lastTime ? (
              <TableRow>
                <TableCell></TableCell>
                <TableCell align="right">{formatTime(lastTime)}</TableCell>
                {penalties.length > 0 ? (
                  <TableCell style={{ color: "red", fontWeight: "bold" }}>
                    {penaltiesPerLap[currentLap]
                      ? `+${penaltiesPerLap[currentLap].toFixed(2)}s`
                      : ""}
                  </TableCell>
                ) : null}
              </TableRow>
            ) : null}
          </TableBody>
        </Table>
      </TableContainer>
    </>
  );
};

export default LapTimes;
