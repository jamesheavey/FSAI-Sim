import React, { useContext, useState, useEffect, useRef } from "react";
import Select from "@material-ui/core/Select";
import MenuItem from "@material-ui/core/MenuItem";
import Title from "../../Components/Title";
import { Context } from "../../App";
import ElementHeader from "../../Components/ElementHeader";
import OpenInNewIcon from "@material-ui/icons/OpenInNew";
import Grid from "@material-ui/core/Grid";
import FormHelperText from "@material-ui/core/FormHelperText";
import FormControl from "@material-ui/core/FormControl";

const getFeeds = async () => {
  const resp = await fetch("/video_feeds");
  return await resp.json();
};

const CameraView = ({ feed, height, width, title, actions }) => {
  const ref = useRef();
  const [selectedFeed, setSelectedFeed] = useState("");
  const [availableFeeds, setAvailableFeeds] = useState([]);
  const [frameKey, setFrameKey] = useState(0);

  const [resolution, setResolution] = useState(0.35);

  useEffect(() => {
    if (feed) {
      return;
    }
    getFeeds().then((feeds) => {
      setAvailableFeeds(feeds);
      setSelectedFeed(feeds[0]);
    });
  }, []);

  const size = Math.min(width - 32, height - 80);

  const resolutionWarningLimit = 0.75;

  return (
    <>
      <ElementHeader
        title={title}
        actions={[
          {
            title: "Open feed",
            icon: <OpenInNewIcon fontSize="small" />,
            handler: () =>
              window.open(`/video_feeds/${feed || selectedFeed}`, "_blank"),
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
          src={`/video_feeds/${feed || selectedFeed}/${resolution.toFixed(2)}`}
          key={frameKey}
          alt="video_feed"
          ref={ref}
          height={size}
          width={size}
        />
        <hr />
        <Grid item container spacing={1}>
          {feed ? null : (
            <Grid item xs={6}>
              <Select
                variant="outlined"
                style={{ width: "100%" }}
                value={selectedFeed}
                // disabled={!(showLaunch || showCarSpawn)}
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
          )}
          <Grid item xs={feed ? 12 : 6}>
            <FormControl
              error={resolution >= resolutionWarningLimit}
              style={{ width: "100%" }}
            >
              <Select
                variant="outlined"
                value={resolution}
                // disabled={!(showLaunch || showCarSpawn)}
                onChange={(e) => {
                  setResolution(e.target.value);
                  setFrameKey(frameKey + 1);
                }}
              >
                {[1.0, 0.75, 0.5, 0.35, 0.2, 0.1].map((m) => (
                  <MenuItem value={m} key={m}>
                    {m}
                  </MenuItem>
                ))}
              </Select>
              {resolution >= resolutionWarningLimit ? (
                <FormHelperText>
                  {`Resolution above ${resolutionWarningLimit} will impact performance`}
                </FormHelperText>
              ) : null}
            </FormControl>
          </Grid>
        </Grid>
      </Grid>
    </>
  );
};

export default CameraView;
