import React, { useContext, useState, useEffect } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";

const ServiceDemo = () => {
  //   const classes = useStyles();
  const context = useContext(Context);

  const [info, setInfo] = useState({ a: "", b: "", result: "" });

  const handleChange = (e) => {
    setInfo({ ...info, [e.target.id]: e.target.value });
  };

  const handleRequestCallback = () => {
    context.actions.send_service_request_callback(info.a, info.b, (result) => {
      setInfo({ ...info, result: result });
    });
  };

  useEffect(() => {
    // console.log("context", context);
  }, [context]);

  const handleRequestState = () => {
    context.actions.send_service_request(info.a, info.b);
  };

  return (
    <>
      <Title>Service Demo</Title>
      <Grid container spacing={1} style={{ width: "100%" }}>
        <Grid item style={{ width: "100%" }}>
          <TextField
            id="a"
            label="Number A"
            type="number"
            variant="outlined"
            value={info.a}
            onChange={handleChange}
            style={{ width: "100%" }}
          />
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <TextField
            id="b"
            label="Number B"
            type="number"
            variant="outlined"
            value={info.b}
            onChange={handleChange}
            style={{ width: "100%" }}
          />
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleRequestCallback}
            style={{ width: "100%" }}
          >
            Add (via callback)
          </Button>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleRequestState}
            style={{ width: "100%" }}
          >
            Add (via state change)
          </Button>
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <TextField
            id="callback"
            label="Result from callback"
            variant="outlined"
            value={info.result}
            InputProps={{
              readOnly: true,
            }}
            style={{ width: "100%" }}
          />
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <TextField
            id="state"
            label="Result from state context"
            variant="outlined"
            value={context?.state?.service_result || ""}
            InputProps={{
              readOnly: true,
            }}
            style={{ width: "100%" }}
          />
        </Grid>
      </Grid>
    </>
  );
};

export default ServiceDemo;
