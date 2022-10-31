import React, { useContext, useState, useEffect, useRef } from "react";

const getCarCorners = (pose, width = 1.76, length = 4.575) => {
  const forward = {
    x: Math.cos(pose.rotation.yaw),
    y: Math.sin(pose.rotation.yaw),
  };

  const left = { x: forward.y, y: -forward.x };

  const halfWidth = width / 2;
  const halfLength = length / 2;

  const corners = [
    { x: -halfWidth, y: -halfLength },
    { x: -halfWidth, y: halfLength },
    { x: halfWidth, y: halfLength },
    { x: halfWidth, y: -halfLength },
  ];

  return corners.map((c) => ({
    x: pose.position.x + c.y * forward.x + c.x * left.x,
    y: pose.position.y + c.y * forward.y + c.x * left.y,
  }));
};

class Drawer {
  constructor(ctx, scale = 0.8) {
    this.ctx = ctx;

    this.extremes = {};

    this.scale = scale;

    this.width = this.ctx.canvas.width;
    this.height = this.ctx.canvas.height;
    this.minSize = this.width < this.height ? this.width : this.height;
    this.car = null;
    this.path = null;
    this.map = null;
    this.allPoints = [];
    this.drawActions = [];
    this.indexPath = null;
  }

  calculateExtremes(points) {
    const extremes = points
      .map((p) => p.position)
      .reduce(
        (ext, p) => ({
          xmin: p.x < ext.xmin ? p.x : ext.xmin,
          xmax: p.x > ext.xmax ? p.x : ext.xmax,
          ymin: p.y < ext.ymin ? p.y : ext.ymin,
          ymax: p.y > ext.ymax ? p.y : ext.ymax,
        }),
        {
          xmin: Number.POSITIVE_INFINITY,
          xmax: Number.NEGATIVE_INFINITY,
          ymin: Number.POSITIVE_INFINITY,
          ymax: Number.NEGATIVE_INFINITY,
        }
      );

    if (extremes.xmin > extremes.xmax || extremes.ymin > extremes.ymax) {
      return;
    }

    const xsize = extremes.xmax - extremes.xmin;
    const ysize = extremes.ymax - extremes.ymin;
    const xcentre = (extremes.xmax + extremes.xmin) / 2;
    const ycentre = (extremes.ymax + extremes.ymin) / 2;

    this.extremes = {
      ...extremes,
      xsize,
      ysize,
      xcentre,
      ycentre,
      maxSize: xsize > ysize ? xsize : ysize,
    };
  }

  calculateScaling() {
    const x_scale = this.extremes.xsize / this.width;
    const y_scale = this.extremes.ysize / this.height;

    const in_ratio = this.extremes.ysize / this.extremes.xsize;
    const out_ratio = this.height / this.width;

    return in_ratio > out_ratio ? y_scale : x_scale;
  }

  rosPointToCanvasPoint(p) {
    const extremes = this.extremes;

    const normaliser = this.calculateScaling();

    const normalised = {
      x: (p.x - extremes.xcentre) / normaliser,
      y: (p.y - extremes.ycentre) / normaliser,
    };

    return {
      x: normalised.x * this.scale + this.width / 2,
      y: this.height / 2 - normalised.y * this.scale,
    };
  }

  rosDistanceToCanvasDistance(d) {
    return (d / this.extremes.maxSize) * this.scale * this.minSize;
  }

  canvasDistanceToRosDistance(d) {
    return (d / this.minSize / this.scale) * this.extremes.maxSize;
  }

  drawPath(path, colour = "#000000", thickness = 1, segments = []) {
    if (!path) {
      return;
    }
    this.ctx.beginPath();
    this.ctx.strokeStyle = colour;
    this.ctx.lineWidth = thickness;
    this.ctx.setLineDash(segments);
    const rPath = path.map((p) => this.rosPointToCanvasPoint(p.position));
    rPath.map((p) => this.ctx.lineTo(p.x, p.y));
    this.ctx.stroke();
    this.ctx.lineWidth = 1;
    this.ctx.setLineDash([]);
  }

  drawCar(car, color = "#f00") {
    if (!car || !car.position) {
      return;
    }

    this.ctx.beginPath();
    this.ctx.strokeStyle = color;

    const corners = getCarCorners(car.position, car.width, car.length);

    const realCorners = corners.map((c) => this.rosPointToCanvasPoint(c));

    this.ctx.moveTo(realCorners[3].x, realCorners[3].y);
    realCorners.map((p) => this.ctx.lineTo(p.x, p.y));

    this.ctx.stroke();
  }

  drawCone(pose, colour = "#ffffff", radius = 0.2) {
    this.ctx.beginPath();
    this.ctx.strokeStyle = colour;
    this.ctx.fillStyle = colour;
    const point = this.rosPointToCanvasPoint(pose.position);
    this.ctx.arc(
      point.x,
      point.y,
      this.rosDistanceToCanvasDistance(radius),
      0,
      2 * Math.PI
    );
    this.ctx.fill();
  }

  drawX(pose, colour = "#ffffff", radius = 0.2, thickness = 1) {
    const point = this.rosPointToCanvasPoint(pose.position);
    const offset = this.rosDistanceToCanvasDistance(radius) / 2;
    const outlineOffset = offset + 1;

    this.ctx.beginPath();
    this.ctx.lineWidth = thickness + 2;
    this.ctx.strokeStyle = "#000000";
    this.ctx.moveTo(point.x - outlineOffset, point.y - outlineOffset);
    this.ctx.lineTo(point.x + outlineOffset, point.y + outlineOffset);
    this.ctx.moveTo(point.x + outlineOffset, point.y - outlineOffset);
    this.ctx.lineTo(point.x - outlineOffset, point.y + outlineOffset);
    this.ctx.stroke();
    this.ctx.beginPath();
    this.ctx.lineWidth = thickness;
    this.ctx.strokeStyle = colour;
    this.ctx.moveTo(point.x - offset, point.y - offset);
    this.ctx.lineTo(point.x + offset, point.y + offset);
    this.ctx.moveTo(point.x + offset, point.y - offset);
    this.ctx.lineTo(point.x - offset, point.y + offset);
    this.ctx.stroke();
    this.ctx.lineWidth = 1;
  }

  drawCircle(pose, colour = "#ffffff", radius = 0.2, thickness = 1) {
    this.ctx.beginPath();
    this.ctx.strokeStyle = colour;
    this.ctx.lineWidth = thickness;
    const point = this.rosPointToCanvasPoint(pose.position);
    this.ctx.arc(
      point.x,
      point.y,
      this.rosDistanceToCanvasDistance(radius),
      0,
      2 * Math.PI
    );
    this.ctx.stroke();
  }

  drawTriangle(pose, colour = "#ffffff", size = 0.2, thickness = 1) {
    this.ctx.beginPath();
    this.ctx.strokeStyle = colour;
    this.ctx.lineWidth = thickness;
    const point = this.rosPointToCanvasPoint(pose.position);
    const _size = (this.rosDistanceToCanvasDistance(size) * 2) / 3;
    this.ctx.moveTo(point.x + _size, point.y);
    this.ctx.lineTo(
      point.x + Math.cos((Math.PI * 2) / 3) * _size,
      point.y + Math.sin((Math.PI * 2) / 3) * _size
    );
    this.ctx.lineTo(
      point.x + Math.cos((Math.PI * 4) / 3) * _size,
      point.y + Math.sin((Math.PI * 4) / 3) * _size
    );
    this.ctx.lineTo(point.x + _size, point.y);
    this.ctx.stroke();
  }

  drawSquare(pose, colour = "#ffffff", size = 0.2, thickness = 1) {
    this.ctx.beginPath();
    this.ctx.strokeStyle = colour;
    this.ctx.lineWidth = thickness;
    const point = this.rosPointToCanvasPoint(pose.position);
    const _size = this.rosDistanceToCanvasDistance(size) * 0.5;
    this.ctx.moveTo(point.x + _size, point.y + _size);
    this.ctx.lineTo(point.x - _size, point.y + _size);
    this.ctx.lineTo(point.x - _size, point.y - _size);
    this.ctx.lineTo(point.x + _size, point.y - _size);
    this.ctx.lineTo(point.x + _size, point.y + _size);
    this.ctx.stroke();
  }

  drawMap(map) {
    (map?.blue_cones || []).map((c) => this.drawCone(c, "#0000ff"));
    (map?.yellow_cones || []).map((c) => this.drawCone(c, "#ffcc00"));
    (map?.orange_cones || []).map((c) => this.drawCone(c, "#ff8800"));
  }

  drawMetaMap(map) {
    (map?.blue_cones || []).map((c) => this.drawX(c, "#0000ff", 0.4, 2));
    (map?.yellow_cones || []).map((c) => this.drawX(c, "#ffcc00", 0.4, 2));
    (map?.orange_cones || []).map((c) => this.drawX(c, "#ff8800", 0.4, 2));
  }

  drawRelativePosition(from, to) {
    const fromPoint = this.rosPointToCanvasPoint(from?.position || {});
    const toPoint = this.rosPointToCanvasPoint(to?.position || {});
    this.ctx.beginPath();
    this.ctx.lineWidth = 2;
    this.ctx.strokeStyle = "#FF0000";
    this.ctx.moveTo(fromPoint.x, fromPoint.y);
    this.ctx.lineTo(toPoint.x, toPoint.y);
    this.ctx.stroke();
  }

  drawMetaCollection(collection) {
    const colour = collection?.properties?.colour || "#000";
    const type = collection?.properties?.type || null;
    const points = collection?.points || [];

    const colourLookup = (colour) => {
      const colours = {
        blue: "#0000ff",
        yellow: "#ffcc00",
        orange: "#ff8800",
      };
      return colours[colour] || colour;
    };

    if (type == "cone") {
      points.forEach((point) => {
        const colOverride = colourLookup(point?.properties?.colour || colour);
        this.drawX({ position: point.position }, colOverride, 0.4, 2);
      });
    } else if (type == "line") {
      const colOverride = colourLookup(colour);
      this.drawPath(
        points.map((point) => ({ position: point.position })),
        colOverride
      );
    } else if (type == "circle") {
      points.forEach((point) => {
        const colOverride = colourLookup(point?.properties?.colour || colour);
        const radius = point?.properties?.radius || 0.15;
        this.drawCircle({ position: point.position }, colOverride, radius);
      });
    } else if (type == "point") {
      points.forEach((point) => {
        const colOverride = colourLookup(point?.properties?.colour || colour);
        const radius = point?.properties?.radius || 0.15;
        this.drawCone({ position: point.position }, colOverride, radius);
      });
    } else if (type == "triangle") {
      points.forEach((point) => {
        const colOverride = colourLookup(point?.properties?.colour || colour);
        const size = point?.properties?.size || 0.15;
        this.drawTriangle({ position: point.position }, colOverride, size);
      });
    } else if (type == "square") {
      points.forEach((point) => {
        const colOverride = colourLookup(point?.properties?.colour || colour);
        const size = point?.properties?.size || 0.15;
        this.drawSquare({ position: point.position }, colOverride, size);
      });
    } else {
      console.error("Unknown meta collection type", type, collection);
    }
  }

  draw() {
    this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height);
    this.ctx.fillStyle = "#000000";

    this.calculateExtremes(this.allPoints);
    this.drawActions.forEach((action) => action());
  }

  getPointerLocation(pointer, distLimit = 5) {
    this.calculateExtremes(this.allPoints);

    if (!this.indexPath || !this.indexPath.length) {
      return null;
    }

    const uniqBy = (a, key) => {
      var seen = {};
      return a.filter((item) => {
        var k = key(item);
        return seen.hasOwnProperty(k) ? false : (seen[k] = true);
      });
    };

    const dist = (a, b) =>
      Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));

    let trackDist = 0;
    let prevPoint = null;

    const path = uniqBy(
      this.indexPath.map((p) => {
        const cd = prevPoint
          ? dist(p.position, prevPoint) + trackDist
          : trackDist;
        trackDist = cd;
        prevPoint = p.position;
        return { ...p, cd };
      }),
      (p) => `${p.position.x.toFixed(5)}_${p.position.y.toFixed(5)}`
    );

    const closestPoints = path
      .map((p, i) => ({
        p: this.rosPointToCanvasPoint(p.position),
        cd: p.cd,
        i,
      }))
      .map((p) => ({
        ...p,
        d: this.canvasDistanceToRosDistance(
          dist(p.p, { x: pointer[0], y: pointer[1] })
        ),
      }))
      .sort((a, b) => a.d - b.d)
      .slice(0, 2);

    if (closestPoints.length != 2) {
      return null;
    }

    const totDist = closestPoints[0].d + closestPoints[1].d;

    if (totDist / 2 > distLimit) {
      return null;
    }

    const [i1, i2, d1, d2] = [
      closestPoints[0].cd,
      closestPoints[1].cd,
      closestPoints[0].d,
      closestPoints[1].d,
    ];

    // console.log(i1, i2, d1, d2, trackDist);

    const len = trackDist;

    const offset = i1 < len / 10 || i2 < len / 10 ? len / 2 : 0;

    return (
      (((((i1 + offset) % len) * d2 + ((i2 + offset) % len) * d1) / totDist +
        offset) %
        len) /
      len
    );
  }

  addPath(path) {
    this.allPoints = [...this.allPoints, ...(path || [])];
    this.drawActions = [...this.drawActions, () => this.drawPath(path)];
    return this;
  }

  addCar(car, color = "#f00") {
    this.drawActions = [...this.drawActions, () => this.drawCar(car, color)];
    return this;
  }

  addMap(map) {
    this.allPoints = [
      ...this.allPoints,
      ...(map?.blue_cones || []),
      ...(map?.yellow_cones || []),
      ...(map?.orange_cones || []),
    ];
    this.drawActions = [...this.drawActions, () => this.drawMap(map)];
    return this;
  }

  addMetaMap(map) {
    this.drawActions = [...this.drawActions, () => this.drawMetaMap(map)];
    return this;
  }

  addMetaPath(path) {
    this.drawActions = [
      ...this.drawActions,
      () => this.drawPath(path || [], "#aa0000", 2),
    ];
    return this;
  }

  addColorPath(path, color, width = 1, segments = []) {
    this.allPoints = [...this.allPoints, ...(path || [])];
    this.drawActions = [
      ...this.drawActions,
      () => this.drawPath(path || [], color, width, segments),
    ];
    return this;
  }

  addIndexingPath(path, color, width = 1, segments = []) {
    if (this.indexPath) {
      throw Error("Indexing path already set");
    }
    this.indexPath = path;
    this.addColorPath(path, color, width, segments);
    return this;
  }

  addRelativePosition(from, to) {
    this.drawActions = [
      ...this.drawActions,
      () => this.drawRelativePosition(from, to),
    ];
    return this;
  }

  addColorPaths(paths, color, width = 1, segments = []) {
    this.drawActions = [
      ...this.drawActions,
      ...(paths?.map((path) => () =>
        this.drawPath(path, color, width, segments)
      ) || []),
    ];
    return this;
  }

  addMetadatas(metadatas) {
    this.drawActions = [
      ...this.drawActions,
      ...(metadatas || []).reduce(
        (arr, metadata) => [
          ...arr,
          ...(metadata?.collections?.map((collection) => () =>
            this.drawMetaCollection(collection)
          ) || []),
        ],
        []
      ),
    ];
  }
}

const DrawerCanvas = ({
  scale,
  handleDrawer,
  updaters,
  selected = null,
  setSelected = () => {},
  ...props
}) => {
  const canvasRef = useRef(null);
  const [canvasWidth, setCanvasWidth] = useState(0);

  function mouseHandler(e) {
    const rect = this.getBoundingClientRect();
    const pointer = [e.clientX - rect.left, e.clientY - rect.top];

    const context = canvasRef.current.getContext("2d");
    const drawer = new Drawer(context, scale);
    handleDrawer(drawer);
    setSelected(drawer.getPointerLocation(pointer));
  }

  useEffect(() => {
    const canvas = canvasRef.current;
    canvas.width = canvasWidth;
    canvas.onmousemove = mouseHandler;
    const context = canvas.getContext("2d");

    const drawer = new Drawer(context, scale);
    handleDrawer(drawer);
    drawer.draw();
  }, [...updaters, canvasWidth]);

  useEffect(() => {
    const handleResize = () => {
      setCanvasWidth(canvasRef.current.clientWidth);
    };

    window.addEventListener("resize", handleResize);

    return (_) => window.removeEventListener("resize", handleResize);
  });
  useEffect(() => {
    setCanvasWidth(canvasRef.current.clientWidth);
  }, [canvasRef.current?.clientWidth]);

  return <canvas ref={canvasRef} {...props} />;
};

export default DrawerCanvas;
