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

const OptimumlapTracks = ({}) => {
  const context = useContext(Context);
  const dashboardContext = useContext(DashboardContext);

  const files = context?.state?.options?.optimumlap_files || [];

  const deleteFile = (file) => {
    fetch(`/optimumlap/${file}`, { method: "DELETE" });
  };

  const handleChangeStatus = ({ meta, remove }, status) => {
    if (status === "headers_received") {
      console.log(`${meta.name} uploaded!`);
      remove();
    } else if (status === "aborted") {
      console.log(`${meta.name}, upload failed...`);
    }
  };

  return (
    <>
      <Title>OptimumLap tracks</Title>
      <TableContainer>
        <Table size="small">
          <TableHead>
            <TableRow key={0}>
              <TableCell component="th" scope="row">
                Track name
              </TableCell>
              <TableCell align="right"></TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {files.map((f) => (
              <TableRow key={f}>
                <TableCell component="th" scope="row">
                  {f}
                </TableCell>
                <TableCell align="right">
                  <Tooltip title="Delete track">
                    <IconButton onClick={() => deleteFile(f)}>
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
        getUploadParams={() => ({ url: "/optimumlap" })}
        onChangeStatus={handleChangeStatus}
        maxFiles={1}
        multiple={false}
        canCancel={false}
        inputContent={"Upload file"}
        accept=".OLTra, .csv"
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

export default OptimumlapTracks;
