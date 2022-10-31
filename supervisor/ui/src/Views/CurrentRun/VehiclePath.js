import React, { useContext, useState, useEffect, useRef } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import DrawerCanvas from "../../DrawerCanvas";
import ElementHeader from "../../Components/ElementHeader";
import RunCanvas from "../../Components/RunCanvas";
import MenuIcon from "@material-ui/icons/Menu";
import PathMenu from "../../Components/PathMenu";

const VehiclePath = ({ height, actions, ...props }) => {
  const context = useContext(Context);
  const [menuAnchor, setMenuAnchor] = useState(null);
  const [pathOptions, setPathOptions] = useState({
    path: { label: "Current lap", value: true },
    allPath: { label: "Previous laps", value: true },
    car: { label: "Vehicle position", value: true },
    fuzzedPath: { label: "Vehicle percieved path", value: false },
    fuzzedCar: { label: "Vehicle percieved position", value: false },
    metadata: { label: "Metadata", value: true },
    centerline: { label: "Centerline", value: true },
    racingline: { label: "Racingline", value: true },
  });

  const path = context.accumulators?.vehicle_position || [];
  const map = context.state?.current_run?.map || {};
  const currentCar = context.state?.current_run?.car || {};
  const car = {
    position: path.length > 0 ? path[path.length - 1] : null,
    length: currentCar.length,
    width: currentCar.width,
  };
  const fuzzedPath = context?.accumulators?.fuzzed_position || [];
  const fuzzedCar = {
    position: fuzzedPath.length > 0 ? fuzzedPath[fuzzedPath.length - 1] : null,
    length: currentCar.length,
    width: currentCar.width,
  };
  const driverMetadata = context.dispatchers?.driver_metadata || {};
  const centerline = context.state?.current_run?.eval_map?.centerline || [];
  const racingline = context.state?.current_run?.eval_map?.racingline || [];
  const _rawRacinglinePos = context.dispatchers?.racingline_position || null;
  const racinglinePosition = _rawRacinglinePos
    ? {
        position: {
          x: _rawRacinglinePos.position_on_racingline.x,
          y: _rawRacinglinePos.position_on_racingline.y,
          z: 0,
        },
        rotation: {
          roll: 0,
          pitch: 0,
          yaw: _rawRacinglinePos.racingline_angle,
        },
      }
    : {};
  const previousPaths =
    context.accumulators?.laps?.map((lap) => lap.vehicle_position || []) || [];

  const metadatas = Object.keys(context.dispatchers || {})
    .filter((key) => key.startsWith("driver_metadata_"))
    .map((key) => context.dispatchers[key]);

  // useEffect(() => {
  //   console.log("meta:", metadatas)
  // }, [metadatas])

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
        map={map}
        car={pathOptions.car.value ? car : {}}
        shadowCar={pathOptions.fuzzedCar.value ? fuzzedCar : {}}
        centerline={pathOptions.centerline.value ? centerline : []}
        racingline={pathOptions.racingline.value ? racingline : []}
        driverMetadata={pathOptions.metadata.value ? driverMetadata : {}}
        racinglinePosition={
          pathOptions.racingline.value ? racinglinePosition : null
        }
        previousPaths={pathOptions.allPath.value ? previousPaths : []}
        metadatas={pathOptions.metadata.value ? metadatas : []}
      />
    </>
  );
};

export default VehiclePath;
