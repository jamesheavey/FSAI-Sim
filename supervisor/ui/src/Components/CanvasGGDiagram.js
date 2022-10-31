import React, { useContext, useMemo, useEffect, useRef } from "react";

const drawGG = (ctx, data, maxVal, margin) => {
  const width = ctx.canvas.width;
  const height = ctx.canvas.height;
  const padding = 30;
  const innerPadding = 10;
  const size = Math.min(width - padding, height - padding);
  const centreX = (width + padding) / 2;
  const centreY = (height - padding) / 2;
  const scale = (size / 2 - innerPadding) / (maxVal * margin);
  ctx.clearRect(0, 0, width, height);

  const drawCircle = (x, y, r, color = "#000000") => {
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
  };

  const drawAxis = () => {
    ctx.strokeStyle = "#000";
    ctx.lineWidth = 1.5;

    // Axis
    ctx.beginPath();
    ctx.moveTo(centreX + size / 2, centreY + size / 2);
    ctx.lineTo(centreX - size / 2, centreY + size / 2);
    ctx.lineTo(centreX - size / 2, centreY - size / 2);
    ctx.stroke();

    // x 0 centerline
    ctx.beginPath();
    ctx.strokeStyle = "#aaa";
    ctx.moveTo(centreX - size / 2, centreY);
    ctx.lineTo(centreX + size / 2, centreY);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centreX - size / 2, centreY + size / 2 - innerPadding);
    ctx.lineTo(centreX + size / 2, centreY + size / 2 - innerPadding);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centreX - size / 2, centreY - size / 2 + innerPadding);
    ctx.lineTo(centreX + size / 2, centreY - size / 2 + innerPadding);
    ctx.stroke();

    // y 0 centerline
    ctx.beginPath();
    ctx.moveTo(centreX, centreY - size / 2);
    ctx.lineTo(centreX, centreY + size / 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centreX + size / 2 - innerPadding, centreY - size / 2);
    ctx.lineTo(centreX + size / 2 - innerPadding, centreY + size / 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centreX - size / 2 + innerPadding, centreY - size / 2);
    ctx.lineTo(centreX - size / 2 + innerPadding, centreY + size / 2);
    ctx.stroke();

    // y 0 tick
    ctx.beginPath();
    ctx.strokeStyle = "#000";
    ctx.moveTo(centreX - size / 2 - 2, centreY);
    ctx.lineTo(centreX - size / 2 + 2, centreY);
    ctx.stroke();

    // x 0 tick
    ctx.beginPath();
    ctx.moveTo(centreX, centreY + size / 2 - 2);
    ctx.lineTo(centreX, centreY + size / 2 + 2);
    ctx.stroke();

    // y scale tick
    ctx.beginPath();
    ctx.moveTo(centreX - size / 2 - 2, centreY + size / 2 - innerPadding);
    ctx.lineTo(centreX - size / 2 + 2, centreY + size / 2 - innerPadding);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(centreX - size / 2 - 2, centreY - size / 2 + innerPadding);
    ctx.lineTo(centreX - size / 2 + 2, centreY - size / 2 + innerPadding);
    ctx.stroke();

    // x scale tick
    ctx.beginPath();
    ctx.moveTo(centreX + size / 2 - innerPadding, centreY + size / 2 - 2);
    ctx.lineTo(centreX + size / 2 - innerPadding, centreY + size / 2 + 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(centreX - size / 2 + innerPadding, centreY + size / 2 - 2);
    ctx.lineTo(centreX - size / 2 + innerPadding, centreY + size / 2 + 2);
    ctx.stroke();
  };

  const drawLabels = () => {
    ctx.font = "12px Roboto";
    ctx.textAlign = "center";
    ctx.fillStyle = "#000";
    ctx.fillText(
      "Lateral Acceleration (g)",
      centreX,
      centreY + size / 2 + (padding / 3) * 2
    );
    ctx.font = "10px Roboto";
    ctx.fillText(
      (-maxVal * margin).toFixed(2),
      centreX - size / 2 + innerPadding,
      centreY + size / 2 + padding / 3
    );
    ctx.fillText((0).toFixed(2), centreX, centreY + size / 2 + padding / 3);
    ctx.fillText(
      (maxVal * margin).toFixed(2),
      centreX + size / 2 - innerPadding,
      centreY + size / 2 + padding / 3
    );

    ctx.save();
    ctx.translate(centreX + 5, centreY);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center";
    ctx.font = "12px Roboto";
    ctx.fillText(
      "Longitudinal Acceleration (g)",
      0,
      -size / 2 - (padding / 3) * 2
    );
    ctx.font = "10px Roboto";
    ctx.fillText(
      (-maxVal * margin).toFixed(2),
      -size / 2 + innerPadding,
      -(size / 2 + padding / 3)
    );
    ctx.fillText((0).toFixed(2), 0, -(size / 2 + padding / 3));
    ctx.fillText(
      (maxVal * margin).toFixed(2),
      size / 2 - innerPadding,
      -(size / 2 + padding / 3)
    );
    ctx.restore();
  };

  const drawPoints = (points) => {
    const color = points.color || "#000";
    points.data
      .filter(
        (p) =>
          Math.abs(p.x) / (maxVal * margin) <= 1 &&
          Math.abs(p.y) / (maxVal * margin) <= 1
      )
      .map((p) => ({ x: -p.x * scale + centreX, y: -p.y * scale + centreY }))
      .map((p) => drawCircle(p.x, p.y, 2, color));
  };

  drawAxis();
  data.map((d) => drawPoints(d));
  drawLabels();
};

const CanvasGGDiagram = ({ width, height, data, ...props }) => {
  //   const context = useContext(Context);
  const canvasRef = useRef(null);

  const margin = 1.1;
  //   const { data, maxVal } = getAccelerationData(context);
  const acc = data.filter(d => d.scaler)?.slice(-1)[0]?.data || [];

  const maxVal = useMemo(
    () =>
      Math.round(
        Math.max(
          ...acc.map((a) => Math.abs(a.x)),
          ...acc.map((a) => Math.abs(a.y)),
          0
        ) * 100
      ) / 100,
    [acc]
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    drawGG(ctx, data, maxVal, margin);
  }, [data, width, height]);

  return <canvas ref={canvasRef} height={height} width={width} />;
};

export default CanvasGGDiagram;
