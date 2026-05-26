import "ol/ol.css";

import Feature from "ol/Feature.js";
import type { FeatureLike } from "ol/Feature.js";
import LineString from "ol/geom/LineString.js";
import MultiPoint from "ol/geom/MultiPoint.js";
import Point from "ol/geom/Point.js";
import OlPolygon, { circular } from "ol/geom/Polygon.js";
import ImageLayer from "ol/layer/Image.js";
import BaseLayer from "ol/layer/Base.js";
import TileLayer from "ol/layer/Tile.js";
import VectorLayer from "ol/layer/Vector.js";
import OlMap from "ol/Map.js";
import { fromLonLat } from "ol/proj.js";
import ImageArcGISRest from "ol/source/ImageArcGISRest.js";
import ImageCanvasSource from "ol/source/ImageCanvas.js";
import VectorSource from "ol/source/Vector.js";
import XYZ from "ol/source/XYZ.js";
import CircleStyle from "ol/style/Circle.js";
import Fill from "ol/style/Fill.js";
import RegularShape from "ol/style/RegularShape.js";
import Stroke from "ol/style/Stroke.js";
import Style from "ol/style/Style.js";
import Text from "ol/style/Text.js";
import View from "ol/View.js";
import { Activity, AlertTriangle, Crosshair, Layers, Pause, Play, Square, Trash2, X } from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { Ros } from "roslib";

import type { NextWebUiConfig } from "./config";
import { callSetBoolService, callTriggerService } from "./rosServices";
import { applyTransform, composeTransform, lookupTransform2D, yawFromQuaternion, type Transform2D } from "./tfMath";
import type { AbsolutePose, MapPoint, MowerMapArea, NavSatFix, OccupancyGrid, SlamAlignmentPose } from "./types";
import { useGpsFix } from "./useGpsFix";
import { useGpsStatus } from "./useGpsStatus";
import { useLaserScan } from "./useLaserScan";
import { useMowerMap } from "./useMowerMap";
import { useOccupancyGrid } from "./useOccupancyGrid";
import { useSlamAlignmentStatus } from "./useSlamAlignmentStatus";
import { useSlamManagerStatus } from "./useSlamManagerStatus";
import { useTfFrames } from "./useTfFrames";

const GPS_STALE_MS = 3000;
const TF_STALE_MS = 5000;
const DEFAULT_FOLLOW_ZOOM = 20;
const FLAG_GPS_RTK_FIXED = 2;
const FLAG_GPS_RTK_FLOAT = 4;
const FLAG_GPS_DEAD_RECKONING = 8;
const METERS_PER_DEGREE_LAT = 111_320;

type GpsStateKind = "dead" | "fixed" | "float" | "gps" | "no-fix" | "offline" | "stale" | "waiting";

interface CombinedMapViewProps {
  config: NextWebUiConfig;
  connected: boolean;
  error: string | null;
  now: number;
  ros: Ros | null;
  url: string;
}

interface LayerSettings {
  calibration: boolean;
  gpsAccuracy: boolean;
  labels: boolean;
  liveScan: boolean;
  mowerMap: boolean;
  satellite: boolean;
  slamObstacles: boolean;
}

interface ProjectionAnchor {
  fixed: boolean;
  lat: number;
  localX: number;
  localY: number;
  lon: number;
  updatedAt: number;
}

interface SlamCanvasRenderState {
  anchor: ProjectionAnchor | null;
  canvas: HTMLCanvasElement | null;
  grid: OccupancyGrid | null;
  gridCanvas: HTMLCanvasElement | null;
  opacity: number;
  slamToMap: Transform2D | null;
}

function createSatelliteLayer(config: NextWebUiConfig) {
  if (config.satelliteSourceType === "arcgis-rest") {
    return new ImageLayer({
      source: new ImageArcGISRest({
        attributions: config.satelliteAttribution,
        params: {
          FORMAT: config.satelliteArcGisFormat,
          LAYERS: config.satelliteArcGisLayers,
        },
        ratio: 1.2,
        url: config.satelliteArcGisRestUrl,
      }),
    });
  }

  return new TileLayer({
    source: new XYZ({
      attributions: config.satelliteAttribution,
      maxZoom: config.satelliteMaxZoom,
      url: config.satelliteTileUrl,
    }),
  });
}

function formatNumber(value: number | null | undefined, digits = 1): string {
  if (value === null || value === undefined || !Number.isFinite(value)) {
    return "--";
  }
  return value.toFixed(digits);
}

function formatMeters(value: number | null | undefined, digits = 1): string {
  return value === null || value === undefined || !Number.isFinite(value) ? "--" : `${formatNumber(value, digits)} m`;
}

function formatAge(lastMessageAt: number | null, now: number): string {
  if (!lastMessageAt) {
    return "--";
  }
  return `${formatNumber(Math.max(0, (now - lastMessageAt) / 1000), 1)} s`;
}

function validLonLat(fix: NavSatFix | null): [number, number] | null {
  if (!fix || !Number.isFinite(fix.longitude) || !Number.isFinite(fix.latitude)) {
    return null;
  }
  if (Math.abs(fix.longitude) > 180 || Math.abs(fix.latitude) > 90) {
    return null;
  }
  return [fix.longitude, fix.latitude];
}

function accuracyFromFix(fix: NavSatFix | null): number {
  if (!fix || fix.position_covariance_type === 0) {
    return Number.NaN;
  }

  const eastVariance = fix.position_covariance[0];
  const northVariance = fix.position_covariance[4];
  if (!Number.isFinite(eastVariance) || !Number.isFinite(northVariance)) {
    return Number.NaN;
  }

  return Math.sqrt(Math.max(eastVariance, northVariance, 0));
}

function accuracyMeters(status: AbsolutePose | null, fix: NavSatFix | null): number {
  if (status && Number.isFinite(status.position_accuracy) && status.position_accuracy >= 0) {
    return status.position_accuracy;
  }
  return accuracyFromFix(fix);
}

function gpsStateLabel(kind: GpsStateKind): string {
  switch (kind) {
    case "dead":
      return "Dead reckoning";
    case "fixed":
      return "RTK fixed";
    case "float":
      return "RTK float";
    case "gps":
      return "GPS fix";
    case "no-fix":
      return "No fix";
    case "offline":
      return "ROS offline";
    case "stale":
      return "Stale";
    case "waiting":
      return "Waiting";
  }
}

function gpsStateKind(
  connected: boolean,
  fix: NavSatFix | null,
  status: AbsolutePose | null,
  lastFixAt: number | null,
  now: number,
): GpsStateKind {
  if (!connected) {
    return "offline";
  }
  if (!fix || !validLonLat(fix)) {
    return "waiting";
  }
  if (!lastFixAt || now - lastFixAt > GPS_STALE_MS) {
    return "stale";
  }
  if (fix.status.status < 0) {
    return "no-fix";
  }

  const flags = status?.flags ?? 0;
  if ((flags & FLAG_GPS_RTK_FIXED) !== 0) {
    return "fixed";
  }
  if ((flags & FLAG_GPS_RTK_FLOAT) !== 0) {
    return "float";
  }
  if ((flags & FLAG_GPS_DEAD_RECKONING) !== 0) {
    return "dead";
  }
  return "gps";
}

function managerLabel(connected: boolean, fresh: boolean, mappingEnabled: boolean, slamRunning: boolean): string {
  if (!connected) {
    return "ROS offline";
  }
  if (!fresh) {
    return "Manager waiting";
  }
  if (mappingEnabled && slamRunning) {
    return "Mapping";
  }
  if (mappingEnabled) {
    return "Starting";
  }
  return "Stopped";
}

function alignmentLabel(state: string | null | undefined, aligned: boolean): string {
  if (!state) {
    return "Waiting";
  }
  if (aligned) {
    return "Aligned";
  }
  return state.replace(/_/g, " ");
}

function poseFromAbsolutePose(status: AbsolutePose | null): Transform2D | null {
  if (!status) {
    return null;
  }
  const position = status.pose.pose.position;
  if (!Number.isFinite(position.x) || !Number.isFinite(position.y)) {
    return null;
  }
  return {
    x: position.x,
    y: position.y,
    yaw: yawFromQuaternion(status.pose.pose.orientation),
  };
}

function poseFromAlignmentPose(pose: SlamAlignmentPose | null | undefined): Transform2D | null {
  if (!pose) {
    return null;
  }
  return {
    x: pose.x,
    y: pose.y,
    yaw: pose.yaw,
  };
}

function localToLonLat(anchor: ProjectionAnchor, point: Pick<MapPoint, "x" | "y">): [number, number] | null {
  const cosLat = Math.cos((anchor.lat * Math.PI) / 180);
  if (Math.abs(cosLat) < 0.000001) {
    return null;
  }

  const lat = anchor.lat + (point.y - anchor.localY) / METERS_PER_DEGREE_LAT;
  const lon = anchor.lon + (point.x - anchor.localX) / (METERS_PER_DEGREE_LAT * cosLat);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
    return null;
  }
  return [lon, lat];
}

function localToMercator(anchor: ProjectionAnchor, point: Pick<MapPoint, "x" | "y">): [number, number] | null {
  const lonLat = localToLonLat(anchor, point);
  const coordinate = lonLat ? fromLonLat(lonLat) : null;
  return coordinate ? [coordinate[0], coordinate[1]] : null;
}

function distance2D(a: Pick<MapPoint, "x" | "y"> | null, b: Pick<MapPoint, "x" | "y"> | null): number {
  if (!a || !b) {
    return Number.NaN;
  }
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function closeRing(points: Array<[number, number]>): Array<[number, number]> {
  if (points.length === 0) {
    return points;
  }
  const first = points[0];
  const last = points[points.length - 1];
  if (first[0] === last[0] && first[1] === last[1]) {
    return points;
  }
  return [...points, first];
}

function createOccupiedGridCanvas(grid: OccupancyGrid | null): HTMLCanvasElement | null {
  if (!grid || grid.info.width <= 0 || grid.info.height <= 0 || grid.data.length === 0) {
    return null;
  }

  const canvas = document.createElement("canvas");
  canvas.width = grid.info.width;
  canvas.height = grid.info.height;
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return null;
  }

  const image = ctx.createImageData(grid.info.width, grid.info.height);
  for (let y = 0; y < grid.info.height; y += 1) {
    for (let x = 0; x < grid.info.width; x += 1) {
      const sourceIndex = y * grid.info.width + x;
      const targetIndex = ((grid.info.height - 1 - y) * grid.info.width + x) * 4;
      const occupied = (grid.data[sourceIndex] ?? -1) >= 50;
      image.data[targetIndex] = 37;
      image.data[targetIndex + 1] = 61;
      image.data[targetIndex + 2] = 96;
      image.data[targetIndex + 3] = occupied ? 210 : 0;
    }
  }

  ctx.putImageData(image, 0, 0);
  return canvas;
}

function renderSlamCanvas(
  state: SlamCanvasRenderState,
  extent: number[],
  resolution: number,
  pixelRatio: number,
  size: number[],
): HTMLCanvasElement {
  const canvas = state.canvas ?? document.createElement("canvas");
  const width = Math.max(1, Math.ceil(size[0]));
  const height = Math.max(1, Math.ceil(size[1]));
  if (canvas.width !== width) {
    canvas.width = width;
  }
  if (canvas.height !== height) {
    canvas.height = height;
  }

  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return canvas;
  }

  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, width, height);

  if (!state.anchor || !state.grid || !state.gridCanvas || !state.slamToMap) {
    return canvas;
  }

  const grid = state.grid;
  const origin = grid.info.origin.position;
  const gridWidthMeters = grid.info.width * grid.info.resolution;
  const gridHeightMeters = grid.info.height * grid.info.resolution;

  const projectSlamPoint = (point: MapPoint) => {
    const mapPoint = applyTransform(state.slamToMap as Transform2D, point);
    const mercator = localToMercator(state.anchor as ProjectionAnchor, mapPoint);
    if (!mercator) {
      return null;
    }
    return {
      x: ((mercator[0] - extent[0]) / resolution) * pixelRatio,
      y: ((extent[3] - mercator[1]) / resolution) * pixelRatio,
    };
  };

  const topLeft = projectSlamPoint({ x: origin.x, y: origin.y + gridHeightMeters });
  const topRight = projectSlamPoint({ x: origin.x + gridWidthMeters, y: origin.y + gridHeightMeters });
  const bottomLeft = projectSlamPoint({ x: origin.x, y: origin.y });
  if (!topLeft || !topRight || !bottomLeft) {
    return canvas;
  }

  ctx.save();
  ctx.globalAlpha = state.opacity;
  ctx.imageSmoothingEnabled = false;
  ctx.setTransform(
    (topRight.x - topLeft.x) / state.gridCanvas.width,
    (topRight.y - topLeft.y) / state.gridCanvas.width,
    (bottomLeft.x - topLeft.x) / state.gridCanvas.height,
    (bottomLeft.y - topLeft.y) / state.gridCanvas.height,
    topLeft.x,
    topLeft.y,
  );
  ctx.drawImage(state.gridCanvas, 0, 0);
  ctx.restore();

  return canvas;
}

function mowerAreaType(area: MowerMapArea): string {
  return area.properties?.type ?? "draft";
}

function mowerFeatureStyle(feature: FeatureLike): Style | Style[] {
  const kind = feature.get("kind");
  const showLabel = Boolean(feature.get("showLabel"));
  if (kind === "dock") {
    return new Style({
      image: new RegularShape({
        fill: new Fill({ color: "#17202a" }),
        points: 3,
        radius: 12,
        rotation: Math.PI / 2 - Number(feature.get("heading") ?? 0),
        stroke: new Stroke({ color: "#ffffff", width: 2 }),
      }),
      text: showLabel
        ? new Text({
            fill: new Fill({ color: "#17202a" }),
            font: "700 12px Inter, system-ui, sans-serif",
            offsetY: -24,
            stroke: new Stroke({ color: "rgba(255,255,255,0.92)", width: 4 }),
            text: feature.get("label") || "Dock",
          })
        : undefined,
      zIndex: 30,
    });
  }

  const type = feature.get("areaType") as string;
  const palette =
    type === "obstacle"
      ? { fill: "rgba(180, 35, 24, 0.20)", stroke: "rgba(180, 35, 24, 0.88)" }
      : type === "nav"
        ? { fill: "rgba(36, 89, 166, 0.14)", stroke: "rgba(36, 89, 166, 0.84)" }
        : { fill: "rgba(20, 120, 77, 0.14)", stroke: "rgba(20, 120, 77, 0.88)" };

  return new Style({
    fill: new Fill({ color: palette.fill }),
    stroke: new Stroke({ color: palette.stroke, width: type === "obstacle" ? 2.5 : 2 }),
    text:
      showLabel && feature.get("label")
        ? new Text({
            fill: new Fill({ color: "#17202a" }),
            font: "700 12px Inter, system-ui, sans-serif",
            overflow: true,
            stroke: new Stroke({ color: "rgba(255,255,255,0.92)", width: 4 }),
            text: feature.get("label"),
          })
        : undefined,
    zIndex: type === "obstacle" ? 20 : 10,
  });
}

function positionFeatureStyle(feature: FeatureLike): Style | Style[] {
  const kind = feature.get("kind");
  if (kind === "accuracy") {
    return new Style({
      fill: new Fill({ color: "rgba(15, 122, 122, 0.12)" }),
      stroke: new Stroke({ color: "rgba(15, 122, 122, 0.72)", width: 2 }),
    });
  }
  if (kind === "connector") {
    return new Style({
      stroke: new Stroke({ color: "rgba(23, 32, 42, 0.68)", lineDash: [6, 6], width: 2 }),
      zIndex: 35,
    });
  }
  if (kind === "rawAntenna") {
    return new Style({
      image: new CircleStyle({
        fill: new Fill({ color: "#eef9f7" }),
        radius: 5,
        stroke: new Stroke({ color: "#0f7a7a", width: 2 }),
      }),
      zIndex: 36,
    });
  }

  const isLidar = kind === "lidarRobot";
  const color = isLidar ? "#5b4bb7" : "#0f7a7a";
  const label = isLidar ? "LIDAR" : "GPS";
  const yaw = Number(feature.get("yaw") ?? 0);
  const showLabel = Boolean(feature.get("showLabel"));
  return new Style({
    image: new RegularShape({
      fill: new Fill({ color }),
      points: 3,
      radius: 14,
      rotation: Math.PI / 2 - yaw,
      stroke: new Stroke({ color: "#ffffff", width: 2.5 }),
    }),
    text: showLabel
      ? new Text({
          fill: new Fill({ color }),
          font: "800 12px Inter, system-ui, sans-serif",
          offsetY: -25,
          stroke: new Stroke({ color: "rgba(255,255,255,0.94)", width: 4 }),
          text: label,
        })
      : undefined,
    zIndex: isLidar ? 42 : 41,
  });
}

function calibrationFeatureStyle(feature: FeatureLike): Style | Style[] {
  const kind = feature.get("kind");
  if (kind === "calibrationLine") {
    return new Style({
      stroke: new Stroke({ color: "rgba(190, 84, 26, 0.62)", lineDash: [3, 5], width: 1.5 }),
      zIndex: 28,
    });
  }
  if (kind === "calibrationLidar") {
    return new Style({
      image: new CircleStyle({
        fill: new Fill({ color: "rgba(91, 75, 183, 0.88)" }),
        radius: 3.6,
        stroke: new Stroke({ color: "#ffffff", width: 1 }),
      }),
      zIndex: 29,
    });
  }
  return new Style({
    image: new CircleStyle({
      fill: new Fill({ color: "rgba(190, 84, 26, 0.92)" }),
      radius: 3.6,
      stroke: new Stroke({ color: "#ffffff", width: 1 }),
    }),
    zIndex: 30,
  });
}

const scanStyle = new Style({
  image: new CircleStyle({
    fill: new Fill({ color: "rgba(36, 89, 166, 0.72)" }),
    radius: 2.4,
  }),
});

function tfState(transform: Transform2D | null): "ok" | "missing" {
  return transform ? "ok" : "missing";
}

export function CombinedMapView({ config, connected, error, now, ros, url }: CombinedMapViewProps) {
  const [layers, setLayers] = useState<LayerSettings>(() => ({
    calibration: config.defaultLayerCalibration,
    gpsAccuracy: config.defaultLayerGpsAccuracy,
    labels: config.defaultLayerLabels,
    liveScan: config.defaultLayerLiveScan,
    mowerMap: config.defaultLayerMowerMap,
    satellite: config.defaultLayerSatellite,
    slamObstacles: config.defaultLayerSlamObstacles,
  }));
  const [projectionAnchor, setProjectionAnchor] = useState<ProjectionAnchor | null>(null);
  const [commandMessage, setCommandMessage] = useState<string | null>(null);
  const [commandPending, setCommandPending] = useState(false);
  const [warningOpen, setWarningOpen] = useState(false);
  const [mapOpacity, setMapOpacity] = useState(0.42);
  const [scanClampMeters, setScanClampMeters] = useState(12);
  const [paused, setPaused] = useState(false);
  const [mapResetKey, setMapResetKey] = useState(0);

  const mapElementRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<OlMap | null>(null);
  const hasCenteredOnFirstFixRef = useRef(false);
  const mowerSourceRef = useRef<VectorSource | null>(null);
  const positionSourceRef = useRef<VectorSource | null>(null);
  const calibrationSourceRef = useRef<VectorSource | null>(null);
  const scanSourceRef = useRef<VectorSource | null>(null);
  const slamSourceRef = useRef<ImageCanvasSource | null>(null);
  const satelliteLayerRef = useRef<BaseLayer | null>(null);
  const slamLayerRef = useRef<ImageLayer<ImageCanvasSource> | null>(null);
  const calibrationLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const mowerLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const scanLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const slamRenderStateRef = useRef<SlamCanvasRenderState>({
    anchor: null,
    canvas: null,
    grid: null,
    gridCanvas: null,
    opacity: mapOpacity,
    slamToMap: null,
  });

  const { fix, stats: fixStats } = useGpsFix({
    ros,
    topicName: config.gpsFixTopic,
  });
  const { stats: rawPoseStats, status: rawPose } = useGpsStatus({
    ros,
    topicName: config.gpsRawPoseTopic,
  });
  const { stats: fusedPoseStats, status: fusedPose } = useGpsStatus({
    ros,
    topicName: config.gpsFusedPoseTopic,
  });
  const { mapData, stats: mowerMapStats } = useMowerMap({
    ros,
    topicName: config.mowerMapTopic,
  });
  const { grid, stats: slamMapStats } = useOccupancyGrid({
    paused,
    resetKey: mapResetKey,
    ros,
    topicName: config.slamMapTopic,
  });
  const { scan, stats: scanStats } = useLaserScan({
    paused: paused || !layers.liveScan,
    ros,
    throttleMs: 100,
    topicName: config.slamScanTopic,
  });
  const { stats: tfStats, transforms } = useTfFrames({
    ros,
    tfStaticTopic: config.tfStaticTopic,
    tfTopic: config.tfTopic,
  });
  const { stats: managerStats, status: managerStatus } = useSlamManagerStatus({
    ros,
    topicName: config.slamManagerStatusTopic,
  });
  const { stats: alignmentStats, status: alignmentStatus } = useSlamAlignmentStatus({
    ros,
    topicName: config.slamAlignmentStatusTopic,
  });

  const alignmentIsFresh = alignmentStats.lastMessageAt !== null && now - alignmentStats.lastMessageAt <= GPS_STALE_MS;
  const statusMapToSlam = alignmentIsFresh ? poseFromAlignmentPose(alignmentStatus?.transform) : null;
  const statusGpsPose = alignmentIsFresh ? poseFromAlignmentPose(alignmentStatus?.gps_pose) : null;
  const statusLidarPose = alignmentIsFresh ? poseFromAlignmentPose(alignmentStatus?.lidar_pose) : null;
  const tfLookupOptions = useMemo(() => ({ maxAgeMs: TF_STALE_MS, nowMs: now }), [now]);
  const mapToOperationalBase = useMemo(
    () => lookupTransform2D(transforms, "map", "base_link", tfLookupOptions),
    [tfLookupOptions, transforms],
  );
  const fusedPoseTransform = useMemo(() => poseFromAbsolutePose(fusedPose), [fusedPose]);
  const freshFusedPose =
    fusedPoseStats.lastMessageAt !== null && now - fusedPoseStats.lastMessageAt <= GPS_STALE_MS
      ? fusedPoseTransform
      : null;
  const gpsPose = statusGpsPose ?? mapToOperationalBase ?? freshFusedPose;
  const tfMapToSlam = useMemo(
    () => lookupTransform2D(transforms, "map", config.slamMapFrame, tfLookupOptions),
    [config.slamMapFrame, tfLookupOptions, transforms],
  );
  const slamToMap = tfMapToSlam ?? statusMapToSlam;
  const slamToBase = useMemo(
    () => lookupTransform2D(transforms, config.slamMapFrame, config.slamBaseFrame, tfLookupOptions),
    [config.slamBaseFrame, config.slamMapFrame, tfLookupOptions, transforms],
  );
  const tfMapToSlamBase = useMemo(
    () => lookupTransform2D(transforms, "map", config.slamBaseFrame, tfLookupOptions),
    [config.slamBaseFrame, tfLookupOptions, transforms],
  );
  const mapToSlamBase = statusLidarPose ?? tfMapToSlamBase ?? (statusMapToSlam && slamToBase ? composeTransform(statusMapToSlam, slamToBase) : null);
  const scanFrame = scan?.header.frame_id || config.slamScanTopic;
  const tfScanToMap = useMemo(
    () => lookupTransform2D(transforms, "map", scanFrame, tfLookupOptions),
    [scanFrame, tfLookupOptions, transforms],
  );
  const slamToScan = useMemo(
    () => lookupTransform2D(transforms, config.slamMapFrame, scanFrame, tfLookupOptions),
    [config.slamMapFrame, scanFrame, tfLookupOptions, transforms],
  );
  const scanToMap = tfScanToMap ?? (statusMapToSlam && slamToScan ? composeTransform(statusMapToSlam, slamToScan) : null);

  const rawPoseIsFresh = rawPoseStats.lastMessageAt !== null && now - rawPoseStats.lastMessageAt <= GPS_STALE_MS;
  const freshRawPose = rawPoseIsFresh ? rawPose : null;
  const lonLat = validLonLat(fix);
  const accuracy = accuracyMeters(freshRawPose, fix);
  const gpsState = gpsStateKind(connected, fix, freshRawPose, fixStats.lastMessageAt, now);
  const rtkFixedFresh = gpsState === "fixed";
  const managerIsFresh = managerStats.lastMessageAt !== null && now - managerStats.lastMessageAt <= GPS_STALE_MS;
  const mappingEnabled = managerIsFresh && Boolean(managerStatus?.mapping_enabled);
  const slamRunning = managerIsFresh && Boolean(managerStatus?.slam_running);
  const slamState = managerLabel(connected, managerIsFresh, mappingEnabled, slamRunning);
  const separation = distance2D(gpsPose, mapToSlamBase);
  const gridCanvas = useMemo(() => createOccupiedGridCanvas(grid), [grid]);
  const followZoom = Math.min(DEFAULT_FOLLOW_ZOOM, config.satelliteMaxZoom);

  const centerMapOnLatestFix = useCallback(() => {
    const map = mapRef.current;
    if (!map) {
      return;
    }

    const markerCenter = projectionAnchor && gpsPose ? localToMercator(projectionAnchor, gpsPose) : null;
    const center = markerCenter ?? (lonLat ? fromLonLat(lonLat) : null);
    if (!center) {
      return;
    }

    const view = map.getView();
    view.setCenter(center);
    if ((view.getZoom() ?? 0) < followZoom) {
      view.setZoom(followZoom);
    }
  }, [followZoom, gpsPose, lonLat, projectionAnchor]);

  useEffect(() => {
    const rawPoseTransform = poseFromAbsolutePose(rawPose);
    const fixFresh = fixStats.lastMessageAt !== null && now - fixStats.lastMessageAt <= GPS_STALE_MS;
    if (!lonLat || !rawPoseTransform || !rawPoseIsFresh || !fixFresh) {
      return;
    }

    const nextAnchor: ProjectionAnchor = {
      fixed: rtkFixedFresh,
      lat: lonLat[1],
      localX: rawPoseTransform.x,
      localY: rawPoseTransform.y,
      lon: lonLat[0],
      updatedAt: now,
    };

    setProjectionAnchor((current) => {
      if (current?.fixed && !nextAnchor.fixed) {
        return current;
      }
      if (
        current &&
        current.fixed === nextAnchor.fixed &&
        Math.abs(current.localX - nextAnchor.localX) < 0.01 &&
        Math.abs(current.localY - nextAnchor.localY) < 0.01 &&
        Math.abs(current.lon - nextAnchor.lon) < 0.0000001 &&
        Math.abs(current.lat - nextAnchor.lat) < 0.0000001
      ) {
        return current;
      }
      return nextAnchor;
    });
  }, [fixStats.lastMessageAt, lonLat, now, rawPose, rawPoseIsFresh, rtkFixedFresh]);

  useEffect(() => {
    if (!mapElementRef.current) {
      return undefined;
    }

    const mowerSource = new VectorSource();
    const positionSource = new VectorSource();
    const calibrationSource = new VectorSource();
    const scanSource = new VectorSource();
    const slamSource = new ImageCanvasSource({
      canvasFunction: (extent, resolution, pixelRatio, size) =>
        renderSlamCanvas(slamRenderStateRef.current, extent, resolution, pixelRatio, size),
      interpolate: false,
      ratio: 1,
    });

    const satelliteLayer = createSatelliteLayer(config);
    satelliteLayer.setOpacity(0.62);
    satelliteLayer.setVisible(layers.satellite);

    const slamLayer = new ImageLayer({
      opacity: mapOpacity,
      source: slamSource,
      visible: false,
    });
    const mowerLayer = new VectorLayer({
      source: mowerSource,
      style: mowerFeatureStyle,
    });
    const scanLayer = new VectorLayer({
      source: scanSource,
      style: scanStyle,
      visible: false,
    });
    const calibrationLayer = new VectorLayer({
      source: calibrationSource,
      style: calibrationFeatureStyle,
      visible: false,
    });

    const map = new OlMap({
      layers: [
        satelliteLayer,
        slamLayer,
        mowerLayer,
        calibrationLayer,
        scanLayer,
        new VectorLayer({
          source: positionSource,
          style: positionFeatureStyle,
        }),
      ],
      target: mapElementRef.current,
      view: new View({
        center: fromLonLat([0, 0]),
        maxZoom: config.satelliteMaxZoom,
        zoom: 2,
      }),
    });

    mapRef.current = map;
    mowerSourceRef.current = mowerSource;
    positionSourceRef.current = positionSource;
    calibrationSourceRef.current = calibrationSource;
    scanSourceRef.current = scanSource;
    slamSourceRef.current = slamSource;
    satelliteLayerRef.current = satelliteLayer;
    slamLayerRef.current = slamLayer;
    calibrationLayerRef.current = calibrationLayer;
    mowerLayerRef.current = mowerLayer;
    scanLayerRef.current = scanLayer;
    hasCenteredOnFirstFixRef.current = false;
    window.requestAnimationFrame(() => map.updateSize());

    return () => {
      map.setTarget(undefined);
      mapRef.current = null;
      mowerSourceRef.current = null;
      positionSourceRef.current = null;
      calibrationSourceRef.current = null;
      scanSourceRef.current = null;
      slamSourceRef.current = null;
      satelliteLayerRef.current = null;
      slamLayerRef.current = null;
      calibrationLayerRef.current = null;
      mowerLayerRef.current = null;
      scanLayerRef.current = null;
    };
  }, [
    config.satelliteArcGisFormat,
    config.satelliteArcGisLayers,
    config.satelliteArcGisRestUrl,
    config.satelliteAttribution,
    config.satelliteMaxZoom,
    config.satelliteSourceType,
    config.satelliteTileUrl,
  ]);

  useEffect(() => {
    satelliteLayerRef.current?.setVisible(layers.satellite);
  }, [layers.satellite]);

  useEffect(() => {
    const slamCanvas = slamRenderStateRef.current.canvas ?? document.createElement("canvas");
    slamRenderStateRef.current = {
      anchor: projectionAnchor,
      canvas: slamCanvas,
      grid,
      gridCanvas,
      opacity: mapOpacity,
      slamToMap,
    };
    slamLayerRef.current?.setVisible(Boolean(layers.slamObstacles && projectionAnchor && grid && gridCanvas && slamToMap));
    slamLayerRef.current?.setOpacity(mapOpacity);
    slamSourceRef.current?.changed();
  }, [grid, gridCanvas, layers.slamObstacles, mapOpacity, projectionAnchor, slamToMap]);

  useEffect(() => {
    const source = mowerSourceRef.current;
    const layer = mowerLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    layer.setVisible(layers.mowerMap);
    if (!layers.mowerMap || !projectionAnchor || !mapData) {
      return;
    }

    const features: Feature[] = [];
    for (const area of mapData.areas) {
      if (area.properties?.active === false || area.outline.length < 3) {
        continue;
      }
      const coordinates = area.outline
        .map((point) => localToMercator(projectionAnchor, point))
        .filter((point): point is [number, number] => point !== null);
      if (coordinates.length < 3) {
        continue;
      }
      features.push(
        new Feature({
          areaType: mowerAreaType(area),
          geometry: new OlPolygon([closeRing(coordinates)]),
          kind: "area",
          label: area.properties?.name || mowerAreaType(area),
          showLabel: layers.labels,
        }),
      );
    }

    for (const station of mapData.docking_stations) {
      if (station.properties?.active === false) {
        continue;
      }
      const coordinate = localToMercator(projectionAnchor, station.position);
      if (!coordinate) {
        continue;
      }
      features.push(
        new Feature({
          geometry: new Point(coordinate),
          heading: station.heading,
          kind: "dock",
          label: station.properties?.name || "Dock",
          showLabel: layers.labels,
        }),
      );
    }

    source.addFeatures(features);
  }, [layers.labels, layers.mowerMap, mapData, projectionAnchor]);

  useEffect(() => {
    const source = positionSourceRef.current;
    if (!source) {
      return;
    }

    source.clear(true);
    const features: Feature[] = [];

    if (lonLat && layers.gpsAccuracy && Number.isFinite(accuracy) && accuracy > 0) {
      const accuracyGeometry = circular(lonLat, accuracy, 64);
      accuracyGeometry.transform("EPSG:4326", "EPSG:3857");
      features.push(new Feature({ geometry: accuracyGeometry, kind: "accuracy" }));
    }
    if (lonLat && layers.gpsAccuracy) {
      features.push(new Feature({ geometry: new Point(fromLonLat(lonLat)), kind: "rawAntenna" }));
    }
    if (projectionAnchor && gpsPose) {
      const coordinate = localToMercator(projectionAnchor, gpsPose);
      if (coordinate) {
        features.push(
          new Feature({
            geometry: new Point(coordinate),
            kind: "gpsRobot",
            showLabel: layers.labels,
            yaw: gpsPose.yaw,
          }),
        );
      }
    }
    if (projectionAnchor && mapToSlamBase) {
      const coordinate = localToMercator(projectionAnchor, mapToSlamBase);
      if (coordinate) {
        features.push(
          new Feature({
            geometry: new Point(coordinate),
            kind: "lidarRobot",
            showLabel: layers.labels,
            yaw: mapToSlamBase.yaw,
          }),
        );
      }
    }
    if (projectionAnchor && gpsPose && mapToSlamBase && separation > 0.2) {
      const a = localToMercator(projectionAnchor, gpsPose);
      const b = localToMercator(projectionAnchor, mapToSlamBase);
      if (a && b) {
        features.push(new Feature({ geometry: new LineString([a, b]), kind: "connector" }));
      }
    }

    source.addFeatures(features);
    if (!hasCenteredOnFirstFixRef.current && (lonLat || gpsPose)) {
      centerMapOnLatestFix();
      hasCenteredOnFirstFixRef.current = true;
    }
  }, [accuracy, centerMapOnLatestFix, gpsPose, layers.gpsAccuracy, layers.labels, lonLat, mapToSlamBase, projectionAnchor, separation]);

  useEffect(() => {
    const source = calibrationSourceRef.current;
    const layer = calibrationLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    layer.setVisible(layers.calibration);
    if (!layers.calibration || !projectionAnchor || !alignmentStatus?.boundary_pairs?.length) {
      return;
    }

    const features: Feature[] = [];
    const gpsPoints: Array<[number, number]> = [];
    const lidarPoints: Array<[number, number]> = [];
    for (const pair of alignmentStatus.boundary_pairs) {
      const gpsCoordinate = localToMercator(projectionAnchor, pair.gps);
      const lidarCoordinate = localToMercator(projectionAnchor, pair.lidar);
      if (!gpsCoordinate || !lidarCoordinate) {
        continue;
      }
      gpsPoints.push(gpsCoordinate);
      lidarPoints.push(lidarCoordinate);
      if (Number.isFinite(pair.residual_m) && pair.residual_m > 0.08) {
        features.push(new Feature({ geometry: new LineString([gpsCoordinate, lidarCoordinate]), kind: "calibrationLine" }));
      }
    }
    if (gpsPoints.length > 0) {
      features.push(new Feature({ geometry: new MultiPoint(gpsPoints), kind: "calibrationGps" }));
    }
    if (lidarPoints.length > 0) {
      features.push(new Feature({ geometry: new MultiPoint(lidarPoints), kind: "calibrationLidar" }));
    }
    source.addFeatures(features);
  }, [alignmentStatus?.boundary_pairs, layers.calibration, projectionAnchor]);

  useEffect(() => {
    const source = scanSourceRef.current;
    const layer = scanLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    layer.setVisible(layers.liveScan);
    if (!layers.liveScan || !projectionAnchor || !scan || !scanToMap) {
      return;
    }

    const maxRange = Math.min(scanClampMeters, scan.range_max || scanClampMeters);
    const stride = Math.max(1, Math.ceil(scan.ranges.length / 520));
    const points: Array<[number, number]> = [];
    for (let index = 0; index < scan.ranges.length; index += stride) {
      const range = scan.ranges[index];
      if (!Number.isFinite(range) || range < scan.range_min || range > maxRange) {
        continue;
      }
      const angle = scan.angle_min + index * scan.angle_increment;
      const localPoint = applyTransform(scanToMap, {
        x: Math.cos(angle) * range,
        y: Math.sin(angle) * range,
      });
      const coordinate = localToMercator(projectionAnchor, localPoint);
      if (coordinate) {
        points.push(coordinate);
      }
    }

    if (points.length > 0) {
      source.addFeature(new Feature({ geometry: new MultiPoint(points) }));
    }
  }, [layers.liveScan, projectionAnchor, scan, scanClampMeters, scanToMap]);

  async function setMappingEnabled(nextEnabled: boolean, force = false): Promise<void> {
    if (!ros || !connected || commandPending) {
      return;
    }
    if (nextEnabled && !force && !rtkFixedFresh) {
      setWarningOpen(true);
      return;
    }

    setCommandPending(true);
    setCommandMessage(null);
    try {
      const response = await callSetBoolService(ros, config.slamSetMappingService, nextEnabled);
      setCommandMessage(response.message || (nextEnabled ? "Mapping started" : "Mapping stopped"));
      if (!response.success) {
        throw new Error(response.message || "Mapping toggle failed");
      }
    } catch (serviceError) {
      setCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setCommandPending(false);
      setWarningOpen(false);
    }
  }

  async function clearSlamMap(): Promise<void> {
    if (!ros || !connected || commandPending) {
      return;
    }

    setCommandPending(true);
    setCommandMessage(null);
    try {
      const response = await callTriggerService(ros, config.slamClearMapService);
      setCommandMessage(response.message || "Map cleared");
      if (response.success) {
        setMapResetKey((current) => current + 1);
      } else {
        throw new Error(response.message || "Map clear failed");
      }
    } catch (serviceError) {
      setCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setCommandPending(false);
    }
  }

  function setLayer(layer: keyof LayerSettings, enabled: boolean): void {
    setLayers((current) => ({ ...current, [layer]: enabled }));
  }

  const metricValues = useMemo(
    () => ({
      accuracy: Number.isFinite(accuracy) ? `${formatNumber(accuracy, accuracy < 1 ? 2 : 1)} m` : "--",
      alignmentAge: formatAge(alignmentStats.lastMessageAt, now),
      anchor: projectionAnchor ? (projectionAnchor.fixed ? "RTK anchor" : "Approx anchor") : "No anchor",
      boundaryPath: formatMeters(alignmentStatus?.boundary_path_length_m, 1),
      boundarySamples: String(alignmentStatus?.boundary_sample_count ?? 0),
      gpsAge: formatAge(fixStats.lastMessageAt, now),
      lidarAge: formatAge(slamMapStats.lastMessageAt, now),
      maxResidual: formatMeters(alignmentStatus?.residual_max_m, 2),
      p95Residual: formatMeters(alignmentStatus?.residual_p95_m, 2),
      residual: formatMeters(alignmentStatus?.residual_m, 2),
      scale:
        alignmentStatus?.scale_diagnostic === null || alignmentStatus?.scale_diagnostic === undefined
          ? "--"
          : formatNumber(alignmentStatus.scale_diagnostic, 3),
      scanHz: formatNumber(scanStats.hz, 1),
      separation: Number.isFinite(separation) ? `${formatNumber(separation, 2)} m` : "--",
      source: alignmentStatus?.alignment_source?.replace(/_/g, " ") || "--",
      yaw:
        alignmentStatus?.yaw_offset_rad === null || alignmentStatus?.yaw_offset_rad === undefined
          ? "--"
          : `${formatNumber((alignmentStatus.yaw_offset_rad * 180) / Math.PI, 1)} deg`,
    }),
    [
      accuracy,
      alignmentStats.lastMessageAt,
      alignmentStatus?.alignment_source,
      alignmentStatus?.boundary_path_length_m,
      alignmentStatus?.boundary_sample_count,
      alignmentStatus?.residual_m,
      alignmentStatus?.residual_max_m,
      alignmentStatus?.residual_p95_m,
      alignmentStatus?.scale_diagnostic,
      alignmentStatus?.yaw_offset_rad,
      fixStats.lastMessageAt,
      now,
      projectionAnchor,
      scanStats.hz,
      separation,
      slamMapStats.lastMessageAt,
    ],
  );

  return (
    <main className="combined-map-workspace">
      <aside className="combined-map-panel">
        <section className="panel-section">
          <div>
            <span className="eyebrow">Combined map</span>
            <h1>Map</h1>
          </div>
          <p className="subtle">{url}</p>
        </section>

        <div className={`gps-state is-${gpsState}`}>
          <Crosshair size={18} aria-hidden="true" />
          <span>{gpsStateLabel(gpsState)}</span>
        </div>

        <section className="panel-section">
          <div className={`sensor-pill is-${mappingEnabled ? "functioning" : "waiting"}`}>
            <Activity size={18} aria-hidden="true" />
            <span>{slamState}</span>
          </div>
          <div className="control-row control-row-split">
            <button
              className="command-button"
              type="button"
              disabled={!connected || commandPending}
              onClick={() => setMappingEnabled(!mappingEnabled)}
            >
              {mappingEnabled ? <Square size={18} aria-hidden="true" /> : <Play size={18} aria-hidden="true" />}
              <span>{commandPending ? "Working" : mappingEnabled ? "Stop LIDAR" : "Start LIDAR"}</span>
            </button>
            <button className="command-button is-danger" type="button" disabled={!connected || commandPending} onClick={clearSlamMap}>
              <Trash2 size={18} aria-hidden="true" />
              <span>Clear</span>
            </button>
          </div>
          {commandMessage && <div className="status-banner">{commandMessage}</div>}
          {(managerStatus?.last_error || alignmentStatus?.last_error) && (
            <div className="error-banner">{managerStatus?.last_error || alignmentStatus?.last_error}</div>
          )}
        </section>

        <section className="panel-section">
          <div className="layer-heading">
            <Layers size={18} aria-hidden="true" />
            <span>Layers</span>
          </div>
          <label className="layer-toggle">
            <input checked={layers.satellite} type="checkbox" onChange={(event) => setLayer("satellite", event.target.checked)} />
            <span>Satellite Reference</span>
          </label>
          <label className="layer-toggle">
            <input checked={layers.mowerMap} type="checkbox" onChange={(event) => setLayer("mowerMap", event.target.checked)} />
            <span>Mower Map</span>
          </label>
          <label className="layer-toggle">
            <input
              checked={layers.slamObstacles}
              type="checkbox"
              onChange={(event) => setLayer("slamObstacles", event.target.checked)}
            />
            <span>SLAM Obstacles</span>
          </label>
          <label className="layer-toggle">
            <input checked={layers.liveScan} type="checkbox" onChange={(event) => setLayer("liveScan", event.target.checked)} />
            <span>Live Scan</span>
          </label>
          <label className="layer-toggle">
            <input
              checked={layers.calibration}
              type="checkbox"
              onChange={(event) => setLayer("calibration", event.target.checked)}
            />
            <span>Calibration Trace</span>
          </label>
          <label className="layer-toggle">
            <input
              checked={layers.gpsAccuracy}
              type="checkbox"
              onChange={(event) => setLayer("gpsAccuracy", event.target.checked)}
            />
            <span>GPS Accuracy</span>
          </label>
          <label className="layer-toggle">
            <input checked={layers.labels} type="checkbox" onChange={(event) => setLayer("labels", event.target.checked)} />
            <span>Labels</span>
          </label>
        </section>

        <section className="panel-section compact-ranges">
          <div className="control-row">
            <button className="command-button" type="button" onClick={() => setPaused((current) => !current)}>
              {paused ? <Play size={18} aria-hidden="true" /> : <Pause size={18} aria-hidden="true" />}
              <span>{paused ? "Resume" : "Pause"}</span>
            </button>
          </div>
          <label htmlFor="combined-slam-opacity">SLAM opacity</label>
          <div className="range-row">
            <input
              id="combined-slam-opacity"
              min="0.1"
              max="0.85"
              step="0.05"
              type="range"
              value={mapOpacity}
              onChange={(event) => setMapOpacity(Number(event.target.value))}
            />
            <output htmlFor="combined-slam-opacity">{Math.round(mapOpacity * 100)}%</output>
          </div>
          <label htmlFor="combined-scan-range">Scan range</label>
          <div className="range-row">
            <input
              id="combined-scan-range"
              min="1"
              max="18"
              step="0.5"
              type="range"
              value={scanClampMeters}
              onChange={(event) => setScanClampMeters(Number(event.target.value))}
            />
            <output htmlFor="combined-scan-range">{formatNumber(scanClampMeters, 1)} m</output>
          </div>
        </section>

        <section className="metric-grid metric-grid-compact" aria-label="Combined map metrics">
          <div className="metric">
            <span>Alignment</span>
            <strong>{alignmentLabel(alignmentStatus?.state, Boolean(alignmentStatus?.aligned))}</strong>
          </div>
          <div className="metric">
            <span>Separation</span>
            <strong>{metricValues.separation}</strong>
          </div>
          <div className="metric">
            <span>Residual</span>
            <strong>{metricValues.residual}</strong>
          </div>
          <div className="metric">
            <span>Boundary</span>
            <strong>{metricValues.boundarySamples}</strong>
          </div>
          <div className="metric">
            <span>Anchor</span>
            <strong>{metricValues.anchor}</strong>
          </div>
          <div className="metric">
            <span>Scan Hz</span>
            <strong>{metricValues.scanHz}</strong>
          </div>
        </section>

        <section className="panel-section calibration-summary">
          <div className="layer-heading">
            <Crosshair size={18} aria-hidden="true" />
            <span>Calibration</span>
          </div>
          <div className="diagnostic-grid diagnostic-grid-tight">
            <div>
              <span>Source</span>
              <strong>{metricValues.source}</strong>
            </div>
            <div>
              <span>Path</span>
              <strong>{metricValues.boundaryPath}</strong>
            </div>
            <div>
              <span>P95</span>
              <strong>{metricValues.p95Residual}</strong>
            </div>
            <div>
              <span>Max</span>
              <strong>{metricValues.maxResidual}</strong>
            </div>
            <div>
              <span>Yaw</span>
              <strong>{metricValues.yaw}</strong>
            </div>
            <div>
              <span>Scale</span>
              <strong>{metricValues.scale}</strong>
            </div>
          </div>
          {alignmentStatus?.drift_warning && <div className="warning-banner">SLAM drift likely</div>}
        </section>

        <details className="advanced-map-details">
          <summary>Advanced diagnostics</summary>
          <div className="diagnostic-grid">
            <div className={`tf-chip is-${tfState(mapToOperationalBase)}`}>
              <span>{"map -> base_link"}</span>
              <strong>{tfState(mapToOperationalBase)}</strong>
            </div>
            <div className={`tf-chip is-${tfState(slamToMap)}`}>
              <span>{`map -> ${config.slamMapFrame}`}</span>
              <strong>{tfState(slamToMap)}</strong>
            </div>
            <div className={`tf-chip is-${tfState(slamToBase)}`}>
              <span>{`${config.slamMapFrame} -> ${config.slamBaseFrame}`}</span>
              <strong>{tfState(slamToBase)}</strong>
            </div>
            <div>
              <span>Map age</span>
              <strong>{metricValues.lidarAge}</strong>
            </div>
            <div>
              <span>GPS age</span>
              <strong>{metricValues.gpsAge}</strong>
            </div>
            <div>
              <span>Align age</span>
              <strong>{metricValues.alignmentAge}</strong>
            </div>
            <div>
              <span>TF Hz</span>
              <strong>{formatNumber(tfStats.hz, 1)}</strong>
            </div>
            <div>
              <span>Mower map</span>
              <strong>{mowerMapStats.messageCount}</strong>
            </div>
          </div>
        </details>

        {error && <div className="error-banner">{error}</div>}
      </aside>

      <section className="combined-map-stage">
        <div className="combined-map-shell">
          <div className="gps-map" ref={mapElementRef} />
          {!lonLat && !projectionAnchor && (
            <div className="gps-empty-state">
              <strong>No GPS projection anchor</strong>
              <span>{config.gpsFixTopic}</span>
            </div>
          )}
          <button
            aria-label="Center map on latest robot pose"
            className="gps-map-control-button"
            disabled={!lonLat && !gpsPose}
            onClick={centerMapOnLatestFix}
            title="Center map on latest robot pose"
            type="button"
          >
            <Crosshair size={20} aria-hidden="true" />
          </button>
          <div className="map-marker-legend" aria-label="Robot marker legend">
            <span className="legend-marker is-gps">GPS</span>
            <span className="legend-marker is-lidar">LIDAR</span>
            <strong>{metricValues.separation}</strong>
          </div>
          <div className="gps-status-strip combined-status-strip" aria-label="Map status">
            <div>
              <span>Accuracy</span>
              <strong>{metricValues.accuracy}</strong>
            </div>
            <div>
              <span>GPS</span>
              <strong>{gpsStateLabel(gpsState)}</strong>
            </div>
            <div>
              <span>SLAM</span>
              <strong>{slamState}</strong>
            </div>
            <div>
              <span>Alignment</span>
              <strong>{alignmentLabel(alignmentStatus?.state, Boolean(alignmentStatus?.aligned))}</strong>
            </div>
            <div>
              <span>Anchor</span>
              <strong>{metricValues.anchor}</strong>
            </div>
          </div>
        </div>
      </section>

      {warningOpen && (
        <div className="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="lidar-warning-title">
          <div className="warning-modal">
            <div className="warning-modal-header">
              <AlertTriangle size={22} aria-hidden="true" />
              <h2 id="lidar-warning-title">GPS is not RTK fixed</h2>
              <button aria-label="Cancel LIDAR mapping" className="icon-button" type="button" onClick={() => setWarningOpen(false)}>
                <X size={18} aria-hidden="true" />
              </button>
            </div>
            <p>
              LIDAR mapping can start now, but the combined map may stay unaligned or approximate until RTK-fixed GPS samples are
              collected.
            </p>
            <div className="modal-actions">
              <button className="command-button is-secondary" type="button" onClick={() => setWarningOpen(false)}>
                <span>Cancel</span>
              </button>
              <button className="command-button" type="button" onClick={() => setMappingEnabled(true, true)}>
                <span>Start anyway</span>
              </button>
            </div>
          </div>
        </div>
      )}
    </main>
  );
}
