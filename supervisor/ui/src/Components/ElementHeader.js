import React from "react";
import Grid from "@material-ui/core/Grid";
import Title from "./Title";
import IconButton from "@material-ui/core/IconButton";
import Tooltip from "@material-ui/core/Tooltip";
import { useSharedStyles } from "../sharedStyles";

const ElementHeader = ({ title, actions }) => {
  const classes = useSharedStyles();
  return (
    <Grid container direction="row" alignItems="flex-start">
      <Title>{title}</Title>
      <div className={classes.grow} />
      {(actions || []).map((action) => (
        <Tooltip title={action.title} key={action.title}>
          <IconButton onClick={action.handler}>{action.icon}</IconButton>
        </Tooltip>
      ))}
    </Grid>
  );
};

export default ElementHeader;
