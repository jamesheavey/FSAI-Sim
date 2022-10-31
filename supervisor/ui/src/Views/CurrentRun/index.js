import React from "react";
import Grid from "@material-ui/core/Grid";
import Paper from "@material-ui/core/Paper";
import VehiclePath from "./VehiclePath";
import CameraView from "./CameraView";
import TimeController from "./TimeController";
import VelocityChart from "./VelocityChart";
import GGDiagram from "./GGDiagram";
import ConsoleView from "./ConsoleView";
import LapTimes from "./LapTimes";
import CrossTrackChart from "./CrossTrackChart";
import DriverSelector from "../Setup/DriverSelector"
import { useSharedStyles } from "../../sharedStyles";
import ElementGrid from "../../Components/ElementGrid";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import {
  faImage,
  faCar,
  faAlignJustify,
  faPlusSquare,
  faChartLine,
  faListUl,
  faCarSide
} from "@fortawesome/free-solid-svg-icons";
import RotateLeftIcon from "@material-ui/icons/RotateLeft";

export default function CurrentRun() {
  const classes = useSharedStyles();

  const elements = {
    path: {
      title: "Vehicle path",
      component: <VehiclePath />,
      dg: { minH: 6, minW: 2 },
      icon: <FontAwesomeIcon icon={faCar} />,
    },
    cam: {
      title: "Camera feed",
      component: <CameraView />,
      dg: { minH: 10, minW: 2 },
      icon: <FontAwesomeIcon icon={faImage} />,
    },
    simCon: {
      title: "Sim Time",
      component: <TimeController />,
      dg: { minH: 3, minW: 2 },
      icon: <RotateLeftIcon />,
    },
    stdout: {
      title: "Driver output",
      component: <ConsoleView />,
      dg: { minW: 2, minH: 4 },
      icon: <FontAwesomeIcon icon={faAlignJustify} />,
    },
    gg: {
      title: "G-G Diagram",
      component: <GGDiagram />,
      dg: { minW: 2, minH: 9 },
      icon: <FontAwesomeIcon icon={faPlusSquare} />,
    },
    vel: {
      title: "Velocity chart",
      component: <VelocityChart />,
      dg: { minW: 2, minH: 5 },
      icon: <FontAwesomeIcon icon={faChartLine} />,
    },
    cross: {
      title: "RMS Cross Track Error chart",
      component: <CrossTrackChart />,
      dg: { minW: 3, minH: 5 },
      icon: <FontAwesomeIcon icon={faChartLine} />,
    },
    laps: {
      title: "Lap times",
      component: <LapTimes />,
      dg: { minW: 2, minH: 5 },
      icon: <FontAwesomeIcon icon={faListUl} />,
    },
    driver: {
      title: "Driver selector",
      component: <DriverSelector />,
      dg: { minW: 3, minH: 8 },
      icon: <FontAwesomeIcon icon={faCarSide} />,
    },
  };
  const defaultLayout = [
    {
      key: 0,
      dg: { x: 0, y: 0, w: 6, h: 20 },
      element: "path",
    },
    {
      key: 1,
      dg: { x: 6, y: 0, w: 3, h: 14 },
      element: "cam",
    },
    {
      key: 2,
      dg: { x: 10, y: 0, h: 3, w: 2 },
      element: "simCon",
    },
    {
      key: 3,
      dg: { x: 10, y: 3, w: 2, h: 5 },
      element: "laps",
    },
  ];

  return (
    <ElementGrid
      name="CurrentRunLayout"
      elements={elements}
      defaultLayout={defaultLayout}
    />
  );
}
