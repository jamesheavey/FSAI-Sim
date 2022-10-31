import { ReceiptTwoTone } from "@material-ui/icons";
import React, { useContext, useState, useEffect, useRef } from "react";

const padding = 5;
const yTextPadding = 25;
const xTextPadding = 40;
const selectionTolerance = 0.005;

const findSelection = (ctx, data, maxVal, pointer) => {
  const width = ctx.canvas.width;
  const height = ctx.canvas.height;
  if (
    pointer[1] < padding ||
    pointer[1] > height - xTextPadding ||
    pointer[0] < padding ||
    pointer[0] > width - 2 * padding - yTextPadding
  ) {
    return;
  }

  const pointerLocation =
    ((pointer[0] - padding) / (width - (2 * padding + xTextPadding))) *
    maxVal.x;
  const closestData = data
    ?.reduce((arr, d) => [...arr, ...d.data], [])
    ?.map((d) => d.x)
    ?.sort(
      (a, b) => Math.abs(a - pointerLocation) - Math.abs(b - pointerLocation)
    )[0];

  // console.log(pointer, pointerLocation, closestData);
  return closestData;
};

const drawVel = (ctx, data, maxVal, xAxisTitle, selected, yAxisTitle) => {
  const width = ctx.canvas.width;
  const height = ctx.canvas.height;
  ctx.clearRect(0, 0, width, height);
  const yTicks =
    maxVal.y < 1 ? 0.1 : maxVal.y < 10 ? 0.5 : maxVal < 50 ? 2 : maxVal < 200 ? 10 : 50;

  const toChartPoint = (p) => {
    const scaled = {
      x: (p.x / maxVal.x) * (width - (2 * padding + xTextPadding)) + padding,
      y:
        (1 - p.y / maxVal.y) * (height - (2 * padding + yTextPadding)) +
        padding,
    };
    return [scaled.x, scaled.y];
  };

  const drawAxis = () => {
    ctx.strokeStyle = "#555";
    ctx.fillStyle = "#555";
    ctx.lineWidth = 0.5;

    ctx.beginPath();
    ctx.moveTo(...toChartPoint({ x: 0, y: 0 }));
    ctx.lineTo(...toChartPoint({ x: maxVal.x, y: 0 }));
    ctx.stroke();
    ctx.textAlign = "center";
    ctx.font = "12px Roboto";
    ctx.fillText(xAxisTitle, width / 2, height - yTextPadding / 2);

    const drawXTick = (i) => {
      ctx.beginPath();
      const p = toChartPoint({ x: i, y: 0 });
      ctx.moveTo(p[0], p[1] - 2);
      ctx.lineTo(p[0], p[1] + 2);
      ctx.stroke();
    };

    if (maxVal.x < 10) {
      [...Array(maxVal.x).keys()].map(drawXTick);
    } else {
      [...Array(Math.ceil(maxVal.x / 10)).keys()].map((i) => drawXTick(i * 10));
    }

    const drawYTick = (y) => {
      ctx.beginPath();
      const p = toChartPoint({ x: maxVal.x, y: y });
      ctx.moveTo(p[0] - 2, p[1]);
      ctx.lineTo(p[0] + 2, p[1]);
      ctx.stroke();

      ctx.font = "10px Roboto";
      ctx.textAlign = "left";
      ctx.fillText(y.toFixed(2), p[0] + 3, p[1] + 3);
    };

    [...Array(Math.ceil(maxVal.y / yTicks)).keys()].map((y) =>
      drawYTick(y * yTicks)
    );

    ctx.save();
    ctx.translate(width - xTextPadding / 3, height / 2);
    ctx.rotate(Math.PI / 2);
    ctx.textAlign = "center";
    ctx.font = "12px Roboto";
    ctx.fillText(yAxisTitle, 0, 0);
    ctx.restore();
  };

  const drawChart = (line) => {
    ctx.beginPath();
    ctx.lineWidth = 1;
    ctx.strokeStyle = line.color || "#000";
    if (line.data.length <= 0) {
      return;
    }
    ctx.moveTo(...toChartPoint(line.data[0]));
    line.data.slice(1).map((p) => ctx.lineTo(...toChartPoint(p)));
    ctx.stroke();
  };

  const drawSelected = () => {
    if (!selected) {
      return;
    }
    ctx.beginPath();
    ctx.lineWidth = 1.2;
    ctx.strokeStyle = "#333";
    ctx.moveTo(...toChartPoint({ x: selected, y: maxVal.y }));
    ctx.lineTo(...toChartPoint({ x: selected, y: 0 }));
    ctx.stroke();

    const drawCircle = (x, y, r, color = "#000000") => {
      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.arc(x, y, r, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.fill();
    };

    const selectedData = data
      ?.map((d) => ({
        v: d.data
          .filter((p) => Math.abs(p.x - selected) < selectionTolerance)
          .sort(
            (a, b) => Math.abs(a.x - selected) - Math.abs(b.x - selected)
          )[0],
        name: d.id,
        selectable: d.selectable,
      }))
      .filter((p) => p.v && p.selectable);

    selectedData
      ?.map((p) => toChartPoint(p.v))
      ?.forEach((p) => drawCircle(p[0], p[1], 2, "#f00"));

    const drawTooltip = (x, y, width, height) => {
      ctx.beginPath();
      ctx.strokeStyle = "#333";
      ctx.fillStyle = "#fff";
      ctx.rect(x, y, width, height);
      ctx.stroke();
      ctx.fill();

      ctx.font = "12px Roboto";
      ctx.textAlign = "left";
      ctx.fillStyle = "#000";
      selectedData?.forEach((d, i) => {
        ctx.fillText(`${d.name}: ${d.v.y.toFixed(4)}`, x + 3, y + i * 20 + 16);
      });
    };

    if (selectedData.length > 0) {
      const tooltipLocation = toChartPoint({ x: selected, y: 0 });
      const edge = toChartPoint({ x: maxVal.x, y: 0 })[0];
      const tooltipHeight = 20 * selectedData.length + 5;
      const tooltipWidth = 120;

      drawTooltip(
        tooltipLocation[0] + tooltipWidth < edge
          ? tooltipLocation[0] + 3
          : tooltipLocation[0] - tooltipWidth - 3,
        tooltipLocation[1] - tooltipHeight - 3,
        tooltipWidth,
        tooltipHeight
      );
    }
  };

  // console.log(selected);

  drawAxis();
  data.map((line) => drawChart(line));
  drawSelected();
};

const CanvasVelocityChart = ({
  width,
  height,
  data,
  xAxisTitle = "time",
  selected = null,
  yAxisTitle = "velocity (m/s)",
  setSelected = () => {},
}) => {
  const canvasRef = useRef(null);
  // const [pointer, setPointer] = useState();

  const maxVal = {
    y: Math.max(
      ...data.reduce((arr, d) => [...arr, ...d.data.map((p) => p.y)], []),
      0.1
    ),
    x: Math.max(
      ...data.reduce((arr, d) => [...arr, ...d.data.map((p) => p.x)], []),
      1
    ),
  };

  function mouseHandler(e) {
    const rect = this.getBoundingClientRect();
    const pointer = [e.clientX - rect.left, e.clientY - rect.top];
    const ctx = canvasRef.current.getContext("2d");
    const pointedData = findSelection(ctx, data, maxVal, pointer);

    setSelected(pointedData);
  }

  useEffect(() => {
    const canvas = canvasRef.current;
    canvas.onmousemove = mouseHandler;
    const ctx = canvas.getContext("2d");
    drawVel(ctx, data, maxVal, xAxisTitle, selected, yAxisTitle);
  }, [data, width, height, xAxisTitle, selected, yAxisTitle]);

  return <canvas ref={canvasRef} height={height} width={width} />;
};

export default CanvasVelocityChart;
