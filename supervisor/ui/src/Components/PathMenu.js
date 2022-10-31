import React, { useContext, useState, useEffect, useRef } from "react";
import Menu from "@material-ui/core/Menu";
import FormGroup from "@material-ui/core/FormGroup";
import FormControlLabel from "@material-ui/core/FormControlLabel";
import Checkbox from "@material-ui/core/Checkbox";
import Box from "@material-ui/core/Box";

const PathMenu = ({ settings, setSettings, anchorEl, onClose }) => {
  return (
    <Menu
      id="simple-menu"
      anchorEl={anchorEl}
      keepMounted
      open={Boolean(anchorEl)}
      onClose={onClose}
      style={{ transformOrigin: "center bottom" }}
    >
      <Box margin={1}>
        <FormGroup>
          {Object.keys(settings).map((k) => (
            <FormControlLabel
              control={
                <Checkbox
                  checked={settings[k].value}
                  onChange={(e) =>
                    setSettings({
                      ...settings,
                      [k]: { ...settings[k], value: e.target.checked },
                    })
                  }
                  color="primary"
                />
              }
              label={settings[k].label}
              key={k}
            />
          ))}
        </FormGroup>
      </Box>
    </Menu>
  );
};

export default PathMenu;
