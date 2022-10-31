import React, { useState, useContext, useEffect, useRef } from "react";
import Title from "../../Components/Title";
import { Context } from "../../App";
import { DashboardContext } from "../../Dashboard";
import DrawerCanvas from "../../DrawerCanvas";

const CarPreview = ({}) => {
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);

  //   const map = context.state?.current_run?.map || {};

  const carName = dashboardContext.setup?.selectedCar?.name || null;

  return (
    <>
      <Title>Car preview</Title>
      <img src={`/car_preview/${carName}`} style={{width: "100%"}} />
    </>
  );
};

export default CarPreview;
