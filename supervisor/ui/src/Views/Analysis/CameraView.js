import React, { useContext, useState, useEffect, useRef } from "react";
import Select from "@material-ui/core/Select";
import MenuItem from "@material-ui/core/MenuItem";
import Title from "../../Components/Title";
import { Context } from "../../App";
import ElementHeader from "../../Components/ElementHeader";
import OpenInNewIcon from "@material-ui/icons/OpenInNew";
import GetAppIcon from "@material-ui/icons/GetApp";
import Grid from "@material-ui/core/Grid";
import FormHelperText from "@material-ui/core/FormHelperText";
import FormControl from "@material-ui/core/FormControl";
import { getSelectedDataOfType, filterSelectedData } from "./util";
import { downloadBlobFromUrl } from "../../utils";

const CameraView = ({ height, width, report, title, actions }) => {
  const ref = useRef();
  const [selectedFeed, setSelectedFeed] = useState("");
  const [frameKey, setFrameKey] = useState(0);

  const size = Math.min(width - 32, height - 80);

  const frames = filterSelectedData(report, (data) =>
    data.key.startsWith("image")
  ).reduce(
    (obj, frame) => ({
      ...obj,
      [frame.topic]: { id: frame.frame_id, key: frame.frame_key },
    }),
    {}
  );
  const availableFeeds = Object.keys(frames);

  useEffect(() => {
    if (!selectedFeed && availableFeeds.length > 0) {
      setSelectedFeed(availableFeeds[0]);
    }
  }, [availableFeeds, selectedFeed]);

  const downloadSelectedFrame = () => {
    const frame = frames[selectedFeed];
    if (!frame) {
      return;
    }
    console.log(report.id);
    const id = report.id;
    const feed = selectedFeed.replace("/", "_");

    downloadBlobFromUrl(
      `/reports/${report.id}/frame/${frames[selectedFeed]?.key}/${frames[selectedFeed]?.id}`,
      `${id}_${feed}_frame.jpg`
    );
  };

  return (
    <>
      <ElementHeader
        title={title}
        actions={[
          {
            title: "Download feed",
            icon: <GetAppIcon fontSize="small" />,
            handler: () => downloadSelectedFrame(),
          },
          ...(actions || []),
        ]}
      />
      <Grid
        container
        direction="column"
        justify="flex-start"
        alignItems="center"
      >
        <img
          src={
            report?.id &&
            frames &&
            selectedFeed &&
            `/reports/${report.id}/frame/${frames[selectedFeed]?.key}/${frames[selectedFeed]?.id}`
          }
          key={frameKey}
          alt="video_feed"
          ref={ref}
          height={size}
          width={size}
        />
        <hr />
        <Grid item container spacing={1}>
          <Grid item xs={12}>
            <Select
              variant="outlined"
              style={{ width: "100%" }}
              value={selectedFeed}
              disabled={availableFeeds.length <= 0}
              onChange={(e) => {
                setSelectedFeed(e.target.value);
                setFrameKey(frameKey + 1);
              }}
            >
              {availableFeeds.map((m) => (
                <MenuItem value={m} key={m}>
                  {m}
                </MenuItem>
              ))}
            </Select>
          </Grid>
        </Grid>
      </Grid>
    </>
  );
};

export default CameraView;
