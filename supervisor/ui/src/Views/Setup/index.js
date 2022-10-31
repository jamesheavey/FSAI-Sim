import React, { useState, useContext } from "react";
import Grid from "@material-ui/core/Grid";
import Paper from "@material-ui/core/Paper";
import { useSharedStyles } from "../../sharedStyles";
import SimulatorManager from "./SimulatiorManager";
import MapPreview from "./MapPreview";
import OptimumlapTracks from "./OptimumlapTracks";
import RuleConfiguration from "./RuleConfiguration";
import DriverSelector from "./DriverSelector";
import CarPreview from "./CarPreview";
import FuzzConfiguration from "./FuzzConfiguration";
import RandomMapCustomisation from "./RandomMapCustomisation";
import MapFuzzing from "./MapFuzzing";
import SemiStaticMapCustomisation from "./SemiStaticMapCustomisation"
import { DashboardContext } from "../../Dashboard";

export default function Setup() {
  const classes = useSharedStyles();
  const dashboardContext = useContext(DashboardContext);
  const map = dashboardContext.setup?.selectedMap || "";

  const customisations = {
    random: <RandomMapCustomisation />,
    small_track_custom: <SemiStaticMapCustomisation />,
    rectangle_track: <SemiStaticMapCustomisation />,
    circle_track: <SemiStaticMapCustomisation />,
  };

  return (
    <Grid container spacing={1}>
      <Grid container item xs={12} md={3} direction="column" spacing={1}>
        <Grid item>
          <Paper className={classes.paper}>
            <SimulatorManager />
          </Paper>
        </Grid>
        <Grid item>
          <Paper className={classes.paper}>
            <DriverSelector />
          </Paper>
        </Grid>
      </Grid>
      <Grid container item xs={12} md={2} direction="column" spacing={1}>
        <Grid item>
          <Paper className={classes.paper}>
            <MapFuzzing />
          </Paper>
        </Grid>
        {!!customisations[map] && (
          <Grid item>
            <Paper className={classes.paper}>{customisations[map]}</Paper>
          </Grid>
        )}
        <Grid item>
          <Paper className={classes.paper}>
            <OptimumlapTracks />
          </Paper>
        </Grid>
      </Grid>
      <Grid container item xs={12} md={3} direction="column" spacing={1}>
        <Grid item>
          <Paper className={classes.paper}>
            <MapPreview />
          </Paper>
        </Grid>
        <Grid item>
          <Paper className={classes.paper}>
            <CarPreview />
          </Paper>
        </Grid>
      </Grid>
      <Grid container item xs={12} md={4} direction="column" spacing={1}>
        <Grid item>
          <Paper className={classes.paper}>
            <RuleConfiguration />
          </Paper>
        </Grid>
        <Grid item>
          <Paper className={classes.paper}>
            <FuzzConfiguration />
          </Paper>
        </Grid>
      </Grid>
    </Grid>
  );
}
