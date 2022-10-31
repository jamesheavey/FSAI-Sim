import React, { useEffect, useState, useRef } from "react";
import Paper from "@material-ui/core/Paper";
import { useSharedStyles } from "../sharedStyles";
import RGL, { WidthProvider } from "react-grid-layout";
import CloseIcon from "@material-ui/icons/Close";

import { makeStyles } from "@material-ui/core/styles";
import SpeedDial from "@material-ui/lab/SpeedDial";
import SpeedDialIcon from "@material-ui/lab/SpeedDialIcon";
import SpeedDialAction from "@material-ui/lab/SpeedDialAction";

const GridLayout = WidthProvider(RGL);

const useStyles = makeStyles((theme) => ({
  speedDial: {
    position: "fixed",
    bottom: theme.spacing(4),
    right: theme.spacing(4),
  },
  tooltip: {
    fontSize: 20,
  },
}));

const ResizablePaper = ({ title, children, actions }) => {
  const ref = useRef();
  const [size, setSize] = useState({ width: 0, height: 0 });

  const classes = useSharedStyles();
  useEffect(() => {
    setSize({
      width: ref.current?.clientWidth,
      height: ref.current?.clientHeight - 74,
    });
  }, [ref.current?.clientWidth, ref.current?.clientHeight]);

  return (
    <Paper className={classes.paper} style={{ height: "100%" }} ref={ref}>
      {React.Children.map(children, (child) =>
        React.cloneElement(child, { ...size, title, actions })
      )}
    </Paper>
  );
};

const getFromLS = (key) => {
  let ls = {};
  if (global.localStorage) {
    try {
      ls = JSON.parse(global.localStorage.getItem(key)) || null;
    } catch (e) {
      /*Ignore*/
    }
  }
  return ls;
};

const saveToLS = (key, value) => {
  if (global.localStorage) {
    global.localStorage.setItem(key, JSON.stringify(value));
  }
};

const ElementGrid = ({ elements, defaultLayout, name, width }) => {
  const classes = useStyles();
  const [fabOpen, setFabOpen] = useState(false);
  const [layout, _setLayout] = useState(getFromLS(name) || defaultLayout);

  const setLayout = (_layout) => {
    _setLayout(_layout);
    saveToLS(name, _layout);
  };

  const handleLayoutChange = (_layout) => {
    const byKey = _layout.reduce((obj, l) => ({ ...obj, [l.i]: l }), {});
    const newLayout = layout.map((l) => ({ ...l, dg: byKey[l.key] }));
    setLayout(newLayout);
  };

  const handleRemove = (key) => {
    setLayout(layout.filter((l) => l.key !== key));
  };

  const handleOpen = () => {
    setFabOpen(true);
  };

  const handleClose = () => {
    setFabOpen(false);
  };

  const handleAdd = (elemKey) => {
    handleClose();
    const newKey = Math.max(...layout.map((l) => l.key), 0) + 1;
    console.log(elemKey, newKey);
    const element = elements[elemKey];
    setLayout([
      ...layout,
      {
        key: newKey,
        dg: {
          x: Infinity,
          y: Infinity,
          w: element.dg?.w || element.dg?.minW || 1,
          h: element.dg?.h || element.dg?.minH || 1,
        },
        element: elemKey,
      },
    ]);
  };

  return (
    <>
      <GridLayout
        className="layout"
        cols={12}
        rowHeight={30}
        width={width || 1200}
        onLayoutChange={handleLayoutChange}
      >
        {layout.map((l) => (
          <div key={l.key} data-grid={{ ...l.dg, ...elements[l.element].dg }}>
            <ResizablePaper
              title={elements[l.element].title}
              actions={[
                ...(elements[l.element].actions || []),
                ...({ ...l.dg, ...elements[l.element].dg }.static
                  ? []
                  : [
                      {
                        title: "close",
                        icon: <CloseIcon fontSize="small" />,
                        handler: () => handleRemove(l.key),
                      },
                    ]),
              ]}
            >
              {elements[l.element].component}
            </ResizablePaper>
          </div>
        ))}
      </GridLayout>
      <SpeedDial
        ariaLabel="" // DO NOT REMOVE. IT WILL CRASH THE ELEMENT. #LABTHINGS
        className={classes.speedDial}
        icon={<SpeedDialIcon />}
        onClose={handleClose}
        onOpen={handleOpen}
        open={fabOpen}
      >
        {Object.keys(elements)
          .filter((key) => elements[key].icon)
          .map((key) => (
            <SpeedDialAction
              key={key}
              icon={elements[key].icon}
              tooltipTitle={elements[key].title}
              // tooltipOpen
              onClick={() => handleAdd(key)}
              TooltipClasses={classes}
            />
          ))}
      </SpeedDial>
    </>
  );
};

export default ElementGrid;
