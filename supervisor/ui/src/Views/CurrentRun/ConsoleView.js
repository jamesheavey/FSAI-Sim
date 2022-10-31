import React, { useContext, useState, useEffect, useRef } from "react";
import PropTypes from "prop-types";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import IconButton from "@material-ui/core/IconButton";
import GetAppIcon from "@material-ui/icons/GetApp";
import Tooltip from "@material-ui/core/Tooltip";
import ElementHeader from "../../Components/ElementHeader";
import { downloadFile } from "../../utils";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";
import PauseIcon from "@material-ui/icons/Pause";

const ConsoleView = ({ height, title, actions }) => {
  const [paused, setPaused] = useState(null);
  const context = useContext(Context);
  const ref = useRef();
  const limit = 50;

  const stdout = context.accumulators?.driver_stdout || [];

  useEffect(() => {
    if (!paused && ref?.current) {
      ref.current.scrollTop = ref.current.scrollHeight;
    }
  }, [stdout, ref?.current?.scrollTop]);

  const handleDownload = () => {
    if (!stdout) {
      return;
    }
    const body = stdout.join("\n");
    downloadFile(body, `driver_stdout_${Math.round(Date.now() / 1000)}.txt`);
  };

  const show = paused
    ? stdout.slice(paused - limit, paused)
    : stdout.slice(-limit);

  return (
    <>
      <ElementHeader
        title={title}
        actions={[
          paused
            ? {
                title: "Unpause",
                icon: <PlayArrowIcon fontSize="small" />,
                handler: () => setPaused(null),
              }
            : {
                title: "Pause",
                icon: <PauseIcon fontSize="small" />,
                handler: () => setPaused(stdout.length),
              },
          {
            title: "Download log",
            icon: <GetAppIcon fontSize="small" />,
            handler: handleDownload,
          },
          ...(actions || []),
        ]}
      />
      <TextField
        label={`Driver output`}
        multiline
        rows={Math.floor((height - 40) / 14)}
        variant="outlined"
        value={show.join("\n")}
        InputProps={{
          readOnly: true,
          style: { fontSize: 12, fontFamily: "Courier New" },
        }}
        inputRef={ref}
      />
    </>
  );
};

export default ConsoleView;
