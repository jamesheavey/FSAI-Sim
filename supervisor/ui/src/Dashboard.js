import React, {
  useState,
  useContext,
  useEffect,
  useReducer,
  createContext,
} from "react";
import CssBaseline from "@material-ui/core/CssBaseline";
import Container from "@material-ui/core/Container";
import { makeStyles } from "@material-ui/core/styles";
import { Context } from "./App";
import Tabs from "@material-ui/core/Tabs";
import Tab from "@material-ui/core/Tab";
import AppBar from "@material-ui/core/AppBar";
import Toolbar from "@material-ui/core/Toolbar";
import CurrentRun from "./Views/CurrentRun";
import Demo from "./Views/Demo";
import Setup from "./Views/Setup";
import Disconnected from "./Views/Disconnected";
import Analysis from "./Views/Analysis";
import IconButton from "@material-ui/core/IconButton";
import Report from "@material-ui/icons/Report";
import Tooltip from "@material-ui/core/Tooltip";
import Snackbar from "@material-ui/core/Snackbar";
import MuiAlert from "@material-ui/lab/Alert";
import Badge from "@material-ui/core/Badge";

const useStyles = makeStyles((theme) => ({
  root: {
    flexGrow: 1,
  },
  container: {
    paddingTop: theme.spacing(4),
    paddingBottom: theme.spacing(4),
    marginTop: theme.spacing(4),
  },
}));

function Alert(props) {
  return <MuiAlert elevation={6} variant="filled" {...props} />;
}

const simpleReducer = (state, updated) => {
  return { ...state, ...updated };
};

const useDashboard = () => {
  const [setup, dispatchSetup] = useReducer(simpleReducer, {});
  const [error, setError] = useReducer(simpleReducer, {});
  const [analysis, setAnalysis] = useReducer(simpleReducer, {
    seenReports: null,
  });

  const context = useContext(Context);

  const clearError = () => {
    setError({ ...error, open: false });
  };

  const addError = (_error) => {
    setError({ ..._error, open: true });
  };

  useEffect(() => {
    if (analysis.seenReports === null && context.state?.reports !== undefined) {
      setAnalysis({ seenReports: context.state?.reports.map((r) => r.id) });
    }
  }, [context.state?.reports]);

  useEffect(() => {
    const unseen =
      context.state?.reports?.filter(
        (r) => !analysis.seenReports?.includes(r.id)
      )?.length || 0;
    setAnalysis({ count: unseen });
  }, [context.state?.reports, analysis.seenReports]);

  return {
    setup,
    dispatchSetup,
    error,
    addError,
    clearError,
    analysis,
    setAnalysis,
  };
};

export const DashboardContext = createContext();

const Dashboard = () => {
  const classes = useStyles();
  const [selectedTab, setSelectedTab] = useState(0);
  const context = useContext(Context);
  const dashboardContext = useDashboard();

  const disconnected = !context.state?.socket;

  const tabs = [
    { name: "Setup", component: <Setup />, disabled: disconnected },
    {
      name: "Current Run",
      component: <CurrentRun />,
      disabled:
        !context.state?.current_run?.ready ||
        context?.state?.current_run?.gazebo_status != 1 ||
        disconnected,
    },
    {
      name: "Analysis",
      component: <Analysis />,
      disabled: disconnected,
      badge: disconnected ? 0 : dashboardContext.analysis.count,
    },
    // { name: "Demo", component: <Demo />, disabled: disconnected },
  ];

  useEffect(() => {
    if (tabs[selectedTab].disabled) {
      setSelectedTab(0);
    }
  }, [tabs, selectedTab]);

  useEffect(() => {
    console.log("event", context.dispatchers?.events);
    if (context.dispatchers?.events) {
      dashboardContext.addError({
        title: context.dispatchers?.events,
        severity: "warning",
      });
      context.actions.clearEvents();
    }
  }, [context.dispatchers?.events]);

  useEffect(() => {
    if(context?.state?.current_run?.gazebo_status == 2){
      dashboardContext.addError({
        title: "Simulator error",
        severity: "error"
      })
      console.log("Gazebo error");
    }
  }, [context?.state?.current_run?.gazebo_status])

  return (
    <>
      <CssBaseline />
      <Snackbar
        open={dashboardContext.error.open}
        autoHideDuration={6000}
        onClose={dashboardContext.clearError}
        anchorOrigin={{ vertical: "top", horizontal: "right" }}
      >
        <Alert
          onClose={dashboardContext.clearError}
          severity={dashboardContext.error.severity}
        >
          {dashboardContext.error.title}
        </Alert>
      </Snackbar>
      <AppBar position="fixed" color="inherit">
        <Toolbar style={{ minHeight: 48 }}>
          {disconnected && (
            <Tooltip title="Server disconnected">
              <IconButton
                edge="start"
                className={classes.menuButton}
                color="secondary"
                aria-label="menu"
              >
                <Report />
              </IconButton>
            </Tooltip>
          )}
          <Tabs
            value={selectedTab}
            onChange={(e, v) => setSelectedTab(v)}
            indicatorColor="primary"
            textColor="primary"
            centered
            style={{ flexGrow: 1 }}
          >
            {tabs.map((tab) => (
              <Tab
                label={
                  <Badge badgeContent={tab.badge} color="primary">
                    {tab.name}
                  </Badge>
                }
                key={tab.name}
                disabled={tab.disabled}
              />
            ))}
          </Tabs>
        </Toolbar>
      </AppBar>
      <DashboardContext.Provider value={dashboardContext}>
        <Container maxWidth="xl" className={classes.container}>
          <div className={classes.root}>
            {disconnected ? <Disconnected /> : tabs[selectedTab].component}
          </div>
        </Container>
      </DashboardContext.Provider>
    </>
  );
};

export default Dashboard;
