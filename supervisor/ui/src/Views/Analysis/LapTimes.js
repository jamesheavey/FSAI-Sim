import React, { useContext, useState, useEffect, useRef } from "react";
import ElementHeader from "../../Components/ElementHeader";
import Table from "@material-ui/core/Table";
import TableBody from "@material-ui/core/TableBody";
import TableCell from "@material-ui/core/TableCell";
import TableContainer from "@material-ui/core/TableContainer";
import TableRow from "@material-ui/core/TableRow";
import GetAppIcon from "@material-ui/icons/GetApp";
import { downloadFile } from "../../utils";
import { getDataOfType } from "./util";

const LapTimes = ({ title, actions, report }) => {
  const laps = getDataOfType(report, "lap") || [];
  const totalLapTime = laps?.reduce((s, l) => s + l.lap_time, 0);
  const lastTime = (report?.snapshots?.slice(-1)[0]?.time || 0) - totalLapTime;
  const penalties = getDataOfType(report, "penalty") || [];
  const penaltiesPerLap = penalties.reduce(
    (obj, p) => ({ ...obj, [p.lap]: (obj[p.lap] || 0) + p.penalty_time }),
    {}
  );
  const lastLap = Math.max(...laps.map((l) => l.lap_count + 1), 0);

  const handleDownload = () => {
    if (!laps) {
      return;
    }
    const body = [...laps, { lap_count: lastLap, lap_time: lastTime }]
      .map(
        (lap) =>
          `Lap ${lap.lap_count + 1}: ${formatTime(lap.lap_time)}${
            penaltiesPerLap[lap.lap_count]
              ? `; +${penaltiesPerLap[lap.lap_count].toFixed(2)}s`
              : ""
          }`
      )
      .join("\n");
    downloadFile(
      body,
      `${report.id}_lap_times_${Math.round(Date.now() / 1000)}.txt`
    );
  };

  const formatTime = (time) =>
    `${Math.floor(time / 60)
      .toString()
      .padStart(2, "0")}:${(time % 60).toFixed(2).padStart(5, "0")}`;

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
            {(laps || []).map((lap) => (
              <TableRow key={lap.lap_count}>
                <TableCell>{`Lap ${lap.lap_count + 1}`}</TableCell>
                <TableCell align="right">{formatTime(lap.lap_time)}</TableCell>
                {penalties.length > 0 ? (
                  <TableCell style={{ color: "red", fontWeight: "bold" }}>
                    {penaltiesPerLap[lap.lap_count]
                      ? `+${penaltiesPerLap[lap.lap_count].toFixed(2)}s`
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
                    {penaltiesPerLap[lastLap]
                      ? `+${penaltiesPerLap[lastLap].toFixed(2)}s`
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
