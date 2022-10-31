import React, { useContext, useState, useEffect } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Title from "../../Components/Title";
import { Context } from "../../App";
import { DashboardContext } from "../../Dashboard";
import TextField from "@material-ui/core/TextField";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import Select from "@material-ui/core/Select";
import MenuItem from "@material-ui/core/MenuItem";
import FormControlLabel from "@material-ui/core/FormControlLabel";
import Checkbox from "@material-ui/core/Checkbox";
import Table from "@material-ui/core/Table";
import TableBody from "@material-ui/core/TableBody";
import TableCell from "@material-ui/core/TableCell";
import TableContainer from "@material-ui/core/TableContainer";
import TableHead from "@material-ui/core/TableHead";
import TableRow from "@material-ui/core/TableRow";

const round3Decimals = (v) => Math.round(v * 1000) / 1000;

const areRulesEqual = (r1, r2) =>
  r1 &&
  r2 &&
  Object.keys(r1).some((k) =>
    Object.keys(r1[k]).some((l) =>
      l === "angle_limit"
        ? round3Decimals(r1[k][l]) !== round3Decimals(r2[k][l])
        : r1[k][l] !== r2[k][l]
    )
  );

const RuleConfiguration = () => {
  const context = useContext(Context);
  //   const dashboardContext = useContext(DashboardContext);
  // const [hasChanges, setHasChanges] = useState(false);
  const [rules, setRules] = useState(null);
  const [originalRules, setOriginalRules] = useState(null);

  useEffect(() => {
    // console.log(context?.state?.run_rules);
    if (
      (!rules && context?.state?.run_rules) ||
      !areRulesEqual(originalRules, context?.state?.run_rules)
    ) {
      setRules(context?.state?.run_rules);
      setOriginalRules(context?.state?.run_rules);
    }
  }, [context?.state?.run_rules]);

  const handleChange = (change) => {
    setRules({
      ...rules,
      ...Object.keys(change).reduce(
        (obj, key) => ({ ...obj, [key]: { ...rules[key], ...change[key] } }),
        {}
      ),
    });
  };

  const handleSave = () => {
    context?.actions?.setRunRules(rules, (res) => {
      // console.log(res);
      setOriginalRules(rules);
    });
  };

  const hasChanges = areRulesEqual(originalRules, rules);

  return (
    <>
      <Title>Run rules</Title>
      <Grid container spacing={1} style={{ width: "100%" }}>
        <TableContainer>
          <Table size="small">
            <TableHead>
              <TableRow key={0}>
                <TableCell component="th" scope="row">
                  Rule (simulation stops if rule is triggered)
                </TableCell>
                <TableCell align="right"></TableCell>
                <TableCell align="right">Enabled</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              <TableRow>
                <TableCell component="th" scope="row">
                  Maximum distance from centerline / track width
                </TableCell>
                <TableCell component="th" scope="row">
                  <Select
                    variant="outlined"
                    style={{ width: "100%" }}
                    value={rules?.distance?.distance_proportion_limit || 2}
                    disabled={!rules?.distance?.enabled}
                    onChange={(e) =>
                      handleChange({
                        distance: { distance_proportion_limit: e.target.value },
                      })
                    }
                  >
                    {[1, 1.2, 1.5, 2, 2.5, 3, 4].map((v) => (
                      <MenuItem value={v} key={v}>{`${v * 100}%`}</MenuItem>
                    ))}
                  </Select>
                </TableCell>
                <TableCell align="right">
                  <Checkbox
                    checked={!!rules?.distance?.enabled}
                    color="primary"
                    onChange={(e) =>
                      handleChange({
                        distance: { enabled: e.target.checked },
                      })
                    }
                  />
                </TableCell>
              </TableRow>
              <TableRow>
                <TableCell component="th" scope="row">
                  Maximum angle from centerline (reverse protection)
                </TableCell>
                <TableCell component="th" scope="row">
                  <Select
                    variant="outlined"
                    style={{ width: "100%" }}
                    value={round3Decimals(rules?.reverse?.angle_limit)}
                    disabled={!rules?.reverse?.enabled}
                    onChange={(e) =>
                      handleChange({
                        reverse: { angle_limit: e.target.value },
                      })
                    }
                  >
                    {[0.5, 2 / 3, 0.75, 0.85].map((v) => (
                      <MenuItem
                        value={round3Decimals(v * Math.PI)}
                        key={v}
                      >{`${v.toFixed(2)} \u03C0`}</MenuItem>
                    ))}
                  </Select>
                </TableCell>
                <TableCell align="right">
                  <Checkbox
                    checked={!!rules?.reverse?.enabled}
                    color="primary"
                    onChange={(e) =>
                      handleChange({
                        reverse: { enabled: e.target.checked },
                      })
                    }
                  />
                </TableCell>
              </TableRow>
              <TableRow>
                <TableCell component="th" scope="row">
                  Number of laps
                </TableCell>
                <TableCell component="th" scope="row">
                  <Select
                    variant="outlined"
                    style={{ width: "100%" }}
                    value={rules?.lap?.lap_count_limit || 3}
                    disabled={!rules?.lap?.enabled}
                    onChange={(e) =>
                      handleChange({
                        lap: { lap_count_limit: e.target.value },
                      })
                    }
                  >
                    {[...Array(12).keys()].map((v) => (
                      <MenuItem value={v + 1} key={v}>
                        {v + 1}
                      </MenuItem>
                    ))}
                  </Select>
                </TableCell>
                <TableCell align="right">
                  <Checkbox
                    checked={!!rules?.lap?.enabled}
                    color="primary"
                    onChange={(e) =>
                      handleChange({
                        lap: { enabled: e.target.checked },
                      })
                    }
                  />
                </TableCell>
              </TableRow>
              <TableRow>
                <TableCell component="th" scope="row">
                  Time elapsed
                </TableCell>
                <TableCell component="th" scope="row">
                  <Select
                    variant="outlined"
                    style={{ width: "100%" }}
                    disabled={!rules?.time?.enabled}
                    value={rules?.time?.time_limit || 60}
                    onChange={(e) =>
                      handleChange({
                        time: { time_limit: e.target.value },
                      })
                    }
                  >
                    {[30, 60, 90, 120, 180, 240, 300, 600].map((v) => (
                      <MenuItem value={v} key={v}>
                        {`${v}s`}
                      </MenuItem>
                    ))}
                  </Select>
                </TableCell>
                <TableCell align="right">
                  <Checkbox
                    checked={!!rules?.time?.enabled}
                    color="primary"
                    onChange={(e) =>
                      handleChange({
                        time: { enabled: e.target.checked },
                      })
                    }
                  />
                </TableCell>
              </TableRow>
              <TableRow>
                <TableCell component="th" scope="row">
                  Penalty issues
                </TableCell>
                <TableCell component="th" scope="row">
                </TableCell>
                <TableCell align="right">
                  <Checkbox
                    checked={!!rules?.penalty?.enabled}
                    color="primary"
                    onChange={(e) =>
                      handleChange({
                        penalty: { enabled: e.target.checked },
                      })
                    }
                  />
                </TableCell>
              </TableRow>
              <TableRow>
                <TableCell component="th" scope="row">
                  Vehicle not moving for duration
                </TableCell>
                <TableCell component="th" scope="row">
                  <Select
                    variant="outlined"
                    style={{ width: "100%" }}
                    disabled={!rules?.vehicle_unmoving?.enabled}
                    value={rules?.vehicle_unmoving?.time_limit || 10}
                    onChange={(e) =>
                      handleChange({
                        vehicle_unmoving: { time_limit: e.target.value },
                      })
                    }
                  >
                    {[5, 10, 15, 20, 30, 60].map((v) => (
                      <MenuItem value={v} key={v}>
                        {`${v}s`}
                      </MenuItem>
                    ))}
                  </Select>
                </TableCell>
                <TableCell align="right">
                  <Checkbox
                    checked={!!rules?.vehicle_unmoving?.enabled}
                    color="primary"
                    onChange={(e) =>
                      handleChange({
                        vehicle_unmoving: { enabled: e.target.checked },
                      })
                    }
                  />
                </TableCell>
              </TableRow>
            </TableBody>
          </Table>
        </TableContainer>
        <hr />
        <Grid item style={{ width: "100%" }}>
          <Button
            variant="contained"
            color="primary"
            onClick={handleSave}
            style={{ width: "100%" }}
            disabled={!hasChanges}
          >
            Apply
          </Button>
        </Grid>
      </Grid>
    </>
  );
};

export default RuleConfiguration;
