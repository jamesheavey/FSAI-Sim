import React, { useContext, useState, useEffect, useRef } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import ElementHeader from "../../Components/ElementHeader";

const StressTest = ({ topic, ...props }) => {
  const context = useContext(Context);

  const [running, setRunning] = useState({});

  const handleClick = () => {
    setRunning({ start: new Date(), count: 0, avg: 0 });
    context?.actions?.publish("/stress/bandwidth", {
      time: 0.05,
      duration: 10,
      payload: { data: "a".repeat(100000) },
    });
  };

  useEffect(() => {
    // console.log(context.accumulators.stress);
    const data = context.accumulators.stress || [];
    if (data.length > 0) {
      setRunning({
        ...running,
        count: data.length || 0,
        avg: ((data.length || 0) / (new Date() - running.start)) * 1000,
      });
    }
  }, [context.accumulators.stress]);

  return (
    <>
      <ElementHeader {...props} />
      <Grid container spacing={1} style={{ width: "100%" }}>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="secondary"
            onClick={handleClick}
            style={{ width: "100%" }}
          >
            Start
          </Button>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          {running.count} {running.avg}
        </Grid>
      </Grid>
    </>
  );
};

export default StressTest;
