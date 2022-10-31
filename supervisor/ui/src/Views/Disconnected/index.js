import React from "react";
import { makeStyles } from "@material-ui/core/styles";
import Typography from "@material-ui/core/Typography";

const useStyles = makeStyles((theme) => ({
  root: {
    alignItems: "center",
    justifyContent: "center",
    display: 'flex',
    height: "90vh"
  },
  text: {
    color: theme.palette.text.secondary,
  },
}));

const Disconnected = () => {
  const classes = useStyles();

  return (
    <div className={classes.root}>
      <Typography className={classes.text}>Server disconnected</Typography>
    </div>
  );
};

export default Disconnected;
