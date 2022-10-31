import React, { useState, useContext, useEffect, useRef } from "react";
import Title from "../../Components/Title";
import { Context } from "../../App";
import { DashboardContext } from "../../Dashboard";
import DrawerCanvas from "../../DrawerCanvas";

const PreviewCanvas = ({ map, car, ...props }) => {
  return (
    <DrawerCanvas
      scale={0.95}
      handleDrawer={(drawer) => {
        drawer.addMap(map).addCar({ ...car, position: map.start });
      }}
      updaters={[map, car]}
      {...props}
    />
  );
};

const VehiclePath = ({}) => {
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);
  const [selectedMap, setSelectedMap] = useState({});

  useEffect(() => {
    console.log(dashboardContext.setup?.selectedMap);
    setSelectedMap({});
    context.actions.gazebo.getMapPreview(
      dashboardContext.setup?.selectedMap,
      {...dashboardContext.setup?.mapProps, fuzzing: dashboardContext.setup?.mapFuzz},
      (map) => {
        setSelectedMap(map);
        console.log(map);
      }
    );
  }, [dashboardContext.setup?.selectedMap, dashboardContext.setup?.mapProps, dashboardContext.setup?.mapFuzz]);

  // const map = context.state?.current_run?.map || {};

  return (
    <>
      <Title>Track preview</Title>
      <PreviewCanvas
        height={300}
        map={selectedMap}
        car={dashboardContext.setup?.selectedCar || {}}
      />
    </>
  );
};

export default VehiclePath;
