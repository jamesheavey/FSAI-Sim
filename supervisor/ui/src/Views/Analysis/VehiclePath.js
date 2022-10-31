import React, { useContext, useState, useEffect, useRef } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import { DashboardContext } from "../../Dashboard";
import ElementHeader from "../../Components/ElementHeader";
import {
  getSelectedDataOfType,
  getDataOfType,
  filterSelectedData,
} from "./util";
import RunCanvas from "../../Components/RunCanvas";
import MenuIcon from "@material-ui/icons/Menu";
import PathMenu from "../../Components/PathMenu";

const getSelectedPaths = (report, selected, selectionTolerance = 0.005) => {
  if (!selected || !report) {
    return [];
  }

  const selectionByLap = getSelectedDataOfType(report, "position")
    ?.filter((p) => Math.abs(p.location.ratio - selected) < selectionTolerance)
    ?.reduce(
      (obj, p) => ({
        ...obj,
        [p.location.lap]: [...(obj[p.location.lap] || []), p],
      }),
      {}
    );
  return Object.keys(selectionByLap).map((k) =>
    selectionByLap[k].map((p) => ({
      position: { x: p.x, y: p.y },
      rotation: { yaw: p.t },
    }))
  );
};

const VehiclePath = ({
  height,
  report,
  selected,
  setSelected,
  actions,
  ...props
}) => {
  const dashboardContext = useContext(DashboardContext);
  const [menuAnchor, setMenuAnchor] = useState(null);
  const [pathOptions, setPathOptions] = useState({
    path: { label: "Selected path", value: true },
    allPath: { label: "All path", value: true },
    car: { label: "Vehicle position", value: true },
    fuzzedPath: { label: "Vehicle percieved path", value: false },
    fuzzedCar: { label: "Vehicle percieved position", value: false },
    metadata: { label: "Metadata", value: true },
    centerline: { label: "Centerline", value: true },
    racingline: { label: "Racingline", value: true },
  });

  //   const report = dashboardContext?.analysis?.report || {};

  const path = getSelectedDataOfType(report, "position")?.map((p) => ({
    position: { x: p.x, y: p.y },
    rotation: { yaw: p.t },
  }));
  const allPath = getDataOfType(report, "position")?.map((p) => ({
    position: { x: p.x, y: p.y },
    rotation: { yaw: p.t },
  }));
  const fuzzedPath = getSelectedDataOfType(report, "fuzzed_position")?.map(
    (p) => ({
      position: { x: p.x, y: p.y },
      rotation: { yaw: p.t },
    })
  );

  const selectedPaths = getSelectedPaths(report, selected);

  const centerline =
    report?.map?.centerline?.length > 0
      ? [...report.map.centerline, report?.map.centerline[0]]
      : [];
  const racingline = report?.map?.racingline || [];

  const car = {
    position: path.length > 0 ? path[path.length - 1] : null,
    length: report?.car?.length || 1,
    width: report?.car?.width || 1,
  };

  const fuzzedCar = {
    position: fuzzedPath.length > 0 ? fuzzedPath[fuzzedPath.length - 1] : null,
    length: report?.car?.length || 1,
    width: report?.car?.width || 1,
  };

  const metadataByType = filterSelectedData(report, (data) =>
    data.key.startsWith("driver_metadata_")
  ).reduce((obj, r) => ({ ...obj, [r.key]: [r, ...(obj[r.key] || [])] }), {});
  const metadatas = Object.keys(metadataByType).map(
    (key) => metadataByType[key][0].metadata
  );
  // console.log(metadatas);

  const handleOpenMenu = (e) => {
    console.log("open");
    setMenuAnchor(e.currentTarget);
  };

  const handleCloseMenu = () => {
    setMenuAnchor(null);
  };

  return (
    <>
      <ElementHeader
        {...props}
        actions={[
          {
            title: "Options",
            icon: <MenuIcon fontSize="small" />,
            handler: handleOpenMenu,
          },
          ...(actions || []),
        ]}
      />

      <PathMenu
        anchorEl={menuAnchor}
        settings={pathOptions}
        setSettings={setPathOptions}
        onClose={handleCloseMenu}
      />

      <RunCanvas
        height={height || 800}
        path={pathOptions.path.value ? path : []}
        shadowPath={pathOptions.fuzzedPath.value ? fuzzedPath : []}
        map={report?.map}
        car={pathOptions.car.value ? car : {}}
        shadowCar={pathOptions.fuzzedCar.value ? fuzzedCar : {}}
        centerline={pathOptions.centerline.value ? centerline : []}
        racingline={pathOptions.racingline.value ? racingline : []}
        previousPaths={pathOptions.allPath.value ? [allPath] : []}
        metadatas={pathOptions.metadata.value ? metadatas : []}
        selected={selected}
        setSelected={setSelected}
        selectedPaths={selectedPaths}
      />
    </>
  );
};

export default VehiclePath;
