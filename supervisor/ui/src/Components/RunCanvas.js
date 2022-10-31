import React, { useContext, useState, useEffect, useRef } from "react";
import DrawerCanvas from "../DrawerCanvas";

const RunCanvas = ({
  path,
  shadowPath,
  map,
  car,
  shadowCar,
  driverMetadata,
  centerline,
  racingline,
  racinglinePosition,
  previousPaths,
  metadatas,
  selectedPaths,
  ...props
}) => {
  return (
    <DrawerCanvas
      scale={0.95}
      handleDrawer={(drawer) =>
        drawer
          .addMap(map)
          .addIndexingPath(centerline, "#999", 2, [5, 15])
          .addColorPath(racingline, "#f0770e", 2)
          .addColorPaths(previousPaths, "#aaa", 0.5)
          .addColorPath(shadowPath, "#aaf", 2)
          .addCar(shadowCar, "#44f")
          .addPath(path)
          .addColorPaths(selectedPaths, "#f00", 2)
          .addCar(car)
          .addMetaMap(driverMetadata?.map)
          .addMetaPath(driverMetadata?.map?.centerline)
          .addRelativePosition(racinglinePosition, car?.position)
          .addMetadatas(metadatas)
      }
      updaters={[path, map, car, driverMetadata, metadatas]}
      {...props}
    />
  );
};

export default RunCanvas;
