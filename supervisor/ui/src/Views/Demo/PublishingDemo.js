import React, { useContext, useState, useEffect, useRef } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import ElementHeader from "../../Components/ElementHeader";
import AccessAlarmIcon from "@material-ui/icons/AccessAlarm";

const PublishingDemo = ({ topic, title, actions }) => {
  const context = useContext(Context);

  const [data, setData] = useState("");

  const handlePublish = () => {
    context?.actions?.publish(topic, data);
    setData("");
  };

  const handleAlert = () => alert("Well, this is alarming");

  return (
    <>
      <ElementHeader
        title={title}
        actions={[
          {
            title: "Alarming",
            icon: <AccessAlarmIcon fontSize="small" />,
            handler: handleAlert,
          },
          ...actions,
        ]}
      />
      <Grid container spacing={1} style={{ width: "100%" }}>
        <Grid item style={{ width: "100%" }}>
          <TextField
            id="data"
            label="Message"
            variant="outlined"
            value={data}
            onChange={(e) => setData(e.target.value)}
            style={{ width: "100%" }}
          />
        </Grid>
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handlePublish}
            style={{ width: "100%" }}
          >
            {`Publish to ${topic}`}
          </Button>
        </Grid>
      </Grid>
    </>
  );
};

export default PublishingDemo;
