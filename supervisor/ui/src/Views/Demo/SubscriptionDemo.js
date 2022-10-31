import React, { useContext, useState, useEffect, useRef } from "react";
import PropTypes from "prop-types";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import ElementHeader from "../../Components/ElementHeader";

const SubscriptionDemo = ({ height, ...props }) => {
  //   const classes = useStyles();
  const context = useContext(Context);
  const ref = useRef();
  useEffect(() => {
    ref.current.scrollTop = ref.current.scrollHeight;
  }, [context, ref]);

  return (
    <>
      <ElementHeader {...props} />
      <TextField
        id="outlined-multiline-static"
        label={`Demo messages`}
        multiline
        rows={Math.floor((height - 15) / 19.5)}
        // style={{height: "100%"}}
        variant="outlined"
        value={(context.accumulators?.demo || []).join("\n")}
        InputProps={{
          readOnly: true,
        }}
        inputRef={ref}
      />
    </>
  );
};

export default SubscriptionDemo;
