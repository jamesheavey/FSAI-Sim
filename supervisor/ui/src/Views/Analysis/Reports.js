import React, { useState, useContext, useEffect, useRef } from "react";
import Title from "../../Components/Title";
import { Context } from "../../App";
import { DashboardContext } from "../../Dashboard";
import Dropzone from "react-dropzone-uploader";
import Button from "@material-ui/core/Button";
import Grid from "@material-ui/core/Grid";
import Table from "@material-ui/core/Table";
import TableBody from "@material-ui/core/TableBody";
import TableCell from "@material-ui/core/TableCell";
import TableContainer from "@material-ui/core/TableContainer";
import TableHead from "@material-ui/core/TableHead";
import TableRow from "@material-ui/core/TableRow";
import IconButton from "@material-ui/core/IconButton";
import DeleteIcon from "@material-ui/icons/Delete";
import Tooltip from "@material-ui/core/Tooltip";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";
import GetAppIcon from "@material-ui/icons/GetApp";
import ElementHeader from "../../Components/ElementHeader";
import {
  downloadFile,
  durationFormatter,
  downloadBlobFromUrl,
} from "../../utils";

const Reports = ({ setReport, report, disabled, ...props }) => {
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);
  const [loading, setLoading] = useState(false);

  const files = context.state?.reports || [];
  // console.log(files);

  const deleteReport = (id) => {
    fetch(`/reports/${id}`, { method: "DELETE" });
    if (id == report?.id) {
      setReport(null);
    }
  };

  const loadReport = (id) => {
    setReport(null);
    setLoading(true);
    // dashboardContext.setAnalysis({ report: null });
    fetch(`/reports/${id}`)
      .then((result) =>
        result.json().then((json) => {
          setReport(json);
          setLoading(false);
        })
      )
      .catch(() => {
        setLoading(false);
      });
  };

  const downloadReport = (f) =>
    downloadBlobFromUrl(`/reports/${f.id}/zip`, `${f.id}.zip`);

  const handleChangeStatus = ({ meta, remove }, status) => {
    if (status === "headers_received") {
      console.log(`${meta.name} uploaded!`);
      remove();
    } else if (status === "aborted") {
      console.log(`${meta.name}, upload failed...`);
    }
    // console.log(meta, remove, status);
  };

  return (
    <>
      <ElementHeader {...props} />
      <TableContainer style={{ marginBottom: "auto", overflowX: "hidden" }}>
        <Table size="small" stickyHeader>
          <TableHead>
            <TableRow>
              <TableCell component="th" scope="row">
                Track
              </TableCell>
              <TableCell align="left">Car</TableCell>
              <TableCell align="right">Created</TableCell>
              <TableCell align="right"></TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {files
              .sort((a, b) => b.started - a.started)
              .map((f) => (
                <TableRow
                  key={f.id}
                  style={
                    dashboardContext.analysis.seenReports.includes(f.id)
                      ? f.id == report?.id
                        ? { backgroundColor: "#D1FFD4" }
                        : {}
                      : { backgroundColor: "#D1D9FF" }
                  }
                >
                  <TableCell component="th" scope="row">
                    {f.map}
                  </TableCell>
                  <TableCell align="left">{f.car}</TableCell>
                  <Tooltip title={new Date(f.started * 1000).toLocaleString()}>
                    <TableCell align="right">
                      {durationFormatter(Date.now() / 1000 - f.started, true)}
                    </TableCell>
                  </Tooltip>
                  <TableCell align="right">
                    {f.id == report?.id ? (
                      <Tooltip title="Download report">
                        <IconButton
                          size="small"
                          onClick={() => downloadReport(f)}
                          disabled={disabled || loading}
                        >
                          <GetAppIcon fontSize="small" />
                        </IconButton>
                      </Tooltip>
                    ) : (
                      <Tooltip title="Load report">
                        <IconButton
                          size="small"
                          onClick={() => loadReport(f.id)}
                          disabled={disabled || loading}
                        >
                          <PlayArrowIcon fontSize="small" />
                        </IconButton>
                      </Tooltip>
                    )}

                    <Tooltip title="Delete report">
                      <IconButton
                        size="small"
                        onClick={() => deleteReport(f.id)}
                        disabled={disabled || loading}
                      >
                        <DeleteIcon fontSize="small" />
                      </IconButton>
                    </Tooltip>
                  </TableCell>
                </TableRow>
              ))}
          </TableBody>
        </Table>
      </TableContainer>
      <Dropzone
        getUploadParams={() => ({ url: "/reports" })}
        onChangeStatus={handleChangeStatus}
        maxFiles={1}
        multiple={false}
        canCancel={false}
        inputContent={"Upload file (or drag 'n' drop)"}
        accept=".zip"
        styles={{
          dropzone: {
            width: "100%",
            height: 10,
            overflow: "hidden",
            borderColor: "transparent",
          },
          dropzoneActive: {
            borderColor: "green",
            zIndex: 1,
          },
        }}
      />
    </>
  );
};

export default Reports;
