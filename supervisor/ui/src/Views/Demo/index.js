import React, { useEffect, useState, useRef } from "react";
import SubscriptionDemo from "./SubscriptionDemo";
import PublishingDemo from "./PublishingDemo";
import StressTest from "./StressTest";
import VehiclePath from "../CurrentRun/VehiclePath";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import {
  faAlignJustify,
  faCar,
  faUpload,
  faSadTear,
} from "@fortawesome/free-solid-svg-icons";

import ElementGrid from "../../Components/ElementGrid"

export default function Demo() {
  const elements = {
    sub: {
      title: "Subscribtion demo",
      component: <SubscriptionDemo />,
      dg: { minH: 4, minW: 2 },
      icon: <FontAwesomeIcon icon={faAlignJustify} />,
    },
    path: {
      title: "Vehicle path",
      component: <VehiclePath />,
      dg: { minH: 6, minW: 2 },
      icon: <FontAwesomeIcon icon={faCar} />,
    },
    publish: {
      title: "Publishing demo",
      component: <PublishingDemo topic="/topic" />,
      dg: { minH: 5, minW: 2, maxH: 5 },
      icon: <FontAwesomeIcon icon={faUpload} />,
    },
    stress: {
      title: "Stress test",
      component: <StressTest />,
      dg: { w: 2, h: 3, isResizable: false },
      icon: <FontAwesomeIcon icon={faSadTear} />,
    },
  };

  const defaultLayout = [
    {
      key: 0,
      dg: { x: 0, y: 0, w: 4, h: 9 },
      element: "sub",
    },
    {
      key: 1,
      dg: { x: 0, y: 9, w: 4, h: 9 },
      element: "path",
    },
    {
      key: 2,
      dg: { x: 4, y: 0, w: 4, h: 5 },
      element: "publish",
    },
    {
      key: 3,
      dg: { x: 8, y: 0 },
      element: "stress",
    },
  ];

  return (
    <ElementGrid
      elements={elements}
      defaultLayout={defaultLayout}
      name="demoLayout"
    />
  );
}
