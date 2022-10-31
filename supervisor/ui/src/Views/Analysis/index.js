import React, { useContext, useEffect, useRef, useState, useMemo } from "react";
import Grid from "@material-ui/core/Grid";
import Paper from "@material-ui/core/Paper";
import VehiclePath from "./VehiclePath";
import { useSharedStyles } from "../../sharedStyles";
import ElementGrid from "../../Components/ElementGrid";
import { Context } from "../../App";
import { DashboardContext } from "../../Dashboard";
import Reports from "./Reports";
import LapTimes from "./LapTimes";
import TimeSelector from "./TimeSelector";
import VelocityChart from "./VelocityChart";
import GGDiagram from "./GGDiagram";
import ConsoleView from "./ConsoleView";
import CameraView from "./CameraView";
import CrossTrackChart from "./CrossTrackChart"
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import {
  faImage,
  faCar,
  faAlignJustify,
  faPlusSquare,
  faChartLine,
  faListUl,
} from "@fortawesome/free-solid-svg-icons";
import RotateLeftIcon from "@material-ui/icons/RotateLeft";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";

const processReport = async (report) => {
  if (!report) {
    return null;
  }
  if (!report.snapshots) {
    return report;
  }

  // console.log(report);

  const toId = (s) => {
    return `${s.lap}_${s.ratio.toFixed(3)}`;
  };

  let startPosition = null;
  let started = false;

  const datapoints = report.snapshots // If the car starts, only show the bit where it started
    // Take last item per key per snapshot, flatten them to datapoints
    .reduce((arr, snapshot) => {
      const dataByKey = snapshot.data.reduce(
        (obj, d) => ({ ...obj, [d.key]: d }),
        {}
      );
      Object.keys(dataByKey).map((key) => ({
        ...dataByKey[key],
        time: snapshot.time,
      }));

      return [
        ...arr,
        ...Object.keys(dataByKey).map((key) => ({
          ...dataByKey[key],
          time: snapshot.time,
        })),
      ];
    }, [])
    // Index points with position and lap
    .reduce(
      (state, point) => {
        let { array, s } = state;
        if (point.key === "pos_on_map") {
          s = { ...s, ratio: point.data.centerline.ratio };
        } else if (point.key === "lap") {
          s = { ...s, ratio: 0, lap: point.data.lap_count + 1 };
        }
        return {
          array: [...array, { ...point, location: s, id: toId(s) }],
          s,
        };
      },
      { array: [], s: { lap: 0, ratio: 0 } }
    ).array
    // Filter out velocity and acceleration before the car starts
    .filter((p) => {
      if (started) {
        return true;
      }
      if(p.key === "position"){
        if (!startPosition) {
          startPosition = p.data;
        }
        if (
          Math.sqrt(
            Math.pow(p.data.x - startPosition.x, 2) +
              Math.pow(p.data.y - startPosition.y, 2)
          ) > 0.1
        ) {
          started = true;
        }
      }
      return (p.key !== "velocity" && p.key !== "acceleration" && p.key !== "pos_on_map") || started;
    });

  return {
    ...report,
    datapoints,
    // datapointsById
  };
};

const updateTimeSelection = (report, timeRange) =>
  report
    ? {
        ...report,
        selectedSnapshots: report?.snapshots?.filter(
          (s) => s.time >= timeRange[0] && s.time <= timeRange[1]
        ),
        selectedDatapoints: report?.datapoints?.filter(
          (p) => p.time >= timeRange[0] && p.time <= timeRange[1]
        ),
      }
    : null;

export default function Analysis() {
  const classes = useSharedStyles();
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);
  const reportsRef = useRef(context.state?.reports);

  const [report, setReport] = useState(null);
  const [timeRange, setTimeRange] = useState(0);
  const [playing, setPlaying] = useState(false);
  const limitedReport = useMemo(() => updateTimeSelection(report, timeRange), [
    timeRange,
    report,
  ]);
  const [trackSelected, setTrackSelected] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleSetReport = (_report) => {
    setTimeRange([0, _report?.snapshots?.slice(-1)[0]?.time || 0]);
    setPlaying(false);
    setLoading(true);
    processReport(_report).then((result) => {
      setReport(result);
      setLoading(false);
    });
    const id = _report?.id;
    if (id && !dashboardContext.analysis.seenReports.includes(id)) {
      dashboardContext.setAnalysis({
        seenReports: [...dashboardContext.analysis.seenReports, id],
      });
    }
  };

  const elements = {
    reports: {
      title: "Reports",
      component: (
        <Reports
          setReport={handleSetReport}
          report={report}
          disabled={loading}
        />
      ),
      dg: { w: 3, h: 15.6, static: true },
    },
    time: {
      title: "Time selector",
      component: (
        <TimeSelector
          report={report}
          time={timeRange}
          setTime={setTimeRange}
          playing={playing}
          setPlaying={setPlaying}
        />
      ),
      dg: { w: 9, h: 1.6, static: true },
    },
    path: {
      title: "Vehicle path",
      component: (
        <VehiclePath
          report={limitedReport}
          selected={trackSelected}
          setSelected={setTrackSelected}
        />
      ),
      dg: { minH: 6, minW: 2 },
      icon: <FontAwesomeIcon icon={faCar} />,
    },
    laps: {
      title: "Lap times",
      component: <LapTimes report={report} />,
      dg: { minW: 2, minH: 5 },
      icon: <FontAwesomeIcon icon={faListUl} />,
    },
    vel: {
      title: "Velocity chart",
      component: (
        <VelocityChart
          report={limitedReport}
          selected={trackSelected}
          setSelected={setTrackSelected}
        />
      ),
      dg: { minW: 2, minH: 5 },
      icon: <FontAwesomeIcon icon={faChartLine} />,
    },
    cte: {
      title: "RMS Cross Track Error chart",
      component: (
        <CrossTrackChart
          report={limitedReport}
          selected={trackSelected}
          setSelected={setTrackSelected}
        />
      ),
      dg: { minW: 3, minH: 5 },
      icon: <FontAwesomeIcon icon={faChartLine} />,
    },
    gg: {
      title: "G-G Diagram",
      component: <GGDiagram report={limitedReport} selected={trackSelected} />,
      dg: { minW: 2, minH: 9 },
      icon: <FontAwesomeIcon icon={faPlusSquare} />,
    },
    stdout: {
      title: "Driver output",
      component: <ConsoleView report={limitedReport} playing={playing} />,
      dg: { minW: 2, minH: 4 },
      icon: <FontAwesomeIcon icon={faAlignJustify} />,
    },
    camera: {
      title: "Camera feed",
      component: <CameraView report={limitedReport} />,
      dg: { minH: 10, minW: 2 },
      icon: <FontAwesomeIcon icon={faImage} />,
    },
  };
  const defaultLayout = [
    {
      key: 0,
      dg: { x: 9, y: 0 },
      element: "reports",
    },
    {
      key: 1,
      dg: { x: 0, y: 0 },
      element: "time",
    },
  ];

  useEffect(() => {
    console.log("reports", context.state?.reports);
    return () => {
      console.log("unload");
      if (context.state?.reports?.length > 0) {
        dashboardContext.setAnalysis({
          seenReports: reportsRef.current.map((r) => r.id),
        });
      }
    };
  }, []);

  useEffect(() => {
    reportsRef.current = context.state?.reports;
  }, [context.state?.reports]);

  useEffect(() => {
    console.log(dashboardContext.analysis.report);
  }, [dashboardContext.analysis.report]);

  return (
    <ElementGrid
      name="AnalysisLayout"
      elements={elements}
      defaultLayout={defaultLayout}
    />
  );
}
