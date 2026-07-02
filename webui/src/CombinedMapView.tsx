import "ol/ol.css";

import Feature from "ol/Feature.js";
import type { FeatureLike } from "ol/Feature.js";
import { boundingExtent } from "ol/extent.js";
import LineString from "ol/geom/LineString.js";
import MultiPoint from "ol/geom/MultiPoint.js";
import Point from "ol/geom/Point.js";
import OlPolygon, { circular } from "ol/geom/Polygon.js";
import ImageLayer from "ol/layer/Image.js";
import BaseLayer from "ol/layer/Base.js";
import TileLayer from "ol/layer/Tile.js";
import VectorLayer from "ol/layer/Vector.js";
import OlMap from "ol/Map.js";
import DragPan from "ol/interaction/DragPan.js";
import { fromLonLat, toLonLat } from "ol/proj.js";
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
import { Activity, AlertTriangle, ChevronDown, ChevronLeft, ChevronRight, CircleDot, Crosshair, Edit3, Flag, Gamepad2, Hand, Layers, LogOut, Map as MapIcon, Pause, Play, Plus, Route, Save, Square, Trash2, Upload, X } from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState, type FormEvent, type PointerEvent as ReactPointerEvent } from "react";
import { Topic, type Ros } from "roslib";

import type { MowerFootprintPoint, NextWebUiConfig } from "./config";
import { callApplyMapEditService, callApplyRecordingEditService, callCreateMapService, callDeleteMapService, callRenameMapService, callSelectMapService, callSetBoolService, callTriggerService } from "./rosServices";
import { applyTransform, composeTransform, lookupTransform2D, lookupTransform2DAt, yawFromQuaternion, type Transform2D } from "./tfMath";
import type {
  AbsolutePose,
  BoolMessage,
  LocalizationFusionStatus,
  MapEditStroke,
  MapPoint,
  MapSummary,
  MowerInputStatus,
  MowerMapArea,
  NavPath,
  NavSatFix,
  OccupancyGrid,
  RoutePlan,
  RoutePlanPath,
  RoutePlanPose,
  SlamAlignmentPose,
  StringMessage,
} from "./types";
import { parseImportedRoutePlanPayload } from "./routePlan";
import { useAreaRecordingControl, type JoystickCommand } from "./useAreaRecordingControl";
import { useAvailableActions } from "./useAvailableActions";
import { useGpsFix } from "./useGpsFix";
import { useGpsStatus } from "./useGpsStatus";
import { useLaserScan } from "./useLaserScan";
import { useLocalizationConfidenceStatus } from "./useLocalizationConfidenceStatus";
import { useLocalizationFusionStatus } from "./useLocalizationFusionStatus";
import { useMapOverlay } from "./useMapOverlay";
import { useMapCatalog } from "./useMapCatalog";
import { useManualPathRecorderStatus } from "./useManualPathRecorderStatus";
import { useMowerMap } from "./useMowerMap";
import { useNavPath } from "./useNavPath";
import { useOccupancyGrid } from "./useOccupancyGrid";
import { useRobotState } from "./useRobotState";
import { useRoutePlan } from "./useRoutePlan";
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
const ACTION_START_AREA_RECORDING = "mower_logic:idle/start_area_recording";
const ACTION_START_RECORDING = "mower_logic:area_recording/start_recording";
const ACTION_STOP_RECORDING = "mower_logic:area_recording/stop_recording";
const ACTION_FINISH_NAVIGATION_AREA = "mower_logic:area_recording/finish_navigation_area";
const ACTION_FINISH_MOWING_AREA = "mower_logic:area_recording/finish_mowing_area";
const ACTION_FINISH_DISCARD = "mower_logic:area_recording/finish_discard";
const ACTION_EXIT_RECORDING_MODE = "mower_logic:area_recording/exit_recording_mode";
const ACTION_AUTO_COLLECT_ENABLE = "mower_logic:area_recording/auto_point_collecting_enable";
const ACTION_AUTO_COLLECT_DISABLE = "mower_logic:area_recording/auto_point_collecting_disable";
const ACTION_COLLECT_POINT = "mower_logic:area_recording/collect_point";
const EDIT_OPERATION_BRUSH_ADD = 1;
const EDIT_OPERATION_BRUSH_ERASE = 2;
const EDIT_OPERATION_REPLACE_POLYGON = 3;
const ACTUAL_TRACK_MIN_STEP_M = 0.03;
const ACTUAL_TRACK_MAX_POINTS = 3000;

type EditorMode = "paint" | "polygon" | "view";
type PaintTool = "eraser" | "pen";
type PolygonTool = "add" | "delete" | "move";
type BrushSizeKey = "large" | "mower" | "small";

const BRUSH_SIZES: Record<BrushSizeKey, { label: string; value: number }> = {
  small: { label: "Small", value: 0.35 },
  mower: { label: "Mower", value: 0.7 },
  large: { label: "Large", value: 1.2 },
};

type GpsStateKind = "dead" | "fixed" | "float" | "gps" | "no-fix" | "offline" | "stale" | "waiting";

interface CombinedMapViewProps {
  config: NextWebUiConfig;
  connected: boolean;
  error: string | null;
  mowerInputStatus: MowerInputStatus | null;
  now: number;
  routeMode?: boolean;
  ros: Ros | null;
  url: string;
}

interface LayerSettings {
  actualTrack: boolean;
  calibration: boolean;
  gpsAccuracy: boolean;
  labels: boolean;
  liveScan: boolean;
  mbfPath: boolean;
  mowerMap: boolean;
  planProgress: boolean;
  satellite: boolean;
  slamObstacles: boolean;
}

interface RouteLayerSettings {
  actualTrack: boolean;
  basePath: boolean;
  mowerMap: boolean;
  outlineFootprints: boolean;
  satellite: boolean;
  selectedFootprint: boolean;
  toolPath: boolean;
}

type RouteSourceMode = "import" | "live";

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

function formatMapTimestamp(value: string | null | undefined): string {
  if (!value) {
    return "--";
  }
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) {
    return value;
  }
  return date.toLocaleDateString(undefined, { month: "short", day: "numeric" });
}

function formatPercent(value: number | null | undefined): string {
  return value === null || value === undefined || !Number.isFinite(value) ? "--" : `${Math.round(value * 100)}%`;
}

function rosStampMs(stamp: { secs: number; nsecs: number } | undefined): number | null {
  if (!stamp || !Number.isFinite(stamp.secs) || !Number.isFinite(stamp.nsecs)) {
    return null;
  }
  return stamp.secs * 1000 + stamp.nsecs / 1_000_000;
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

function mercatorToLocal(anchor: ProjectionAnchor, coordinate: [number, number]): MapPoint | null {
  const [lon, lat] = toLonLat(coordinate);
  const cosLat = Math.cos((anchor.lat * Math.PI) / 180);
  if (!Number.isFinite(lon) || !Number.isFinite(lat) || Math.abs(cosLat) < 0.000001) {
    return null;
  }
  return {
    x: anchor.localX + (lon - anchor.lon) * METERS_PER_DEGREE_LAT * cosLat,
    y: anchor.localY + (lat - anchor.lat) * METERS_PER_DEGREE_LAT,
  };
}

function transformFootprintPoint(pose: Transform2D, point: MowerFootprintPoint): MapPoint {
  const cosYaw = Math.cos(pose.yaw);
  const sinYaw = Math.sin(pose.yaw);
  return {
    x: pose.x + cosYaw * point[0] - sinYaw * point[1],
    y: pose.y + sinYaw * point[0] + cosYaw * point[1],
  };
}

function footprintToMercatorRing(
  anchor: ProjectionAnchor,
  pose: Transform2D,
  footprint: MowerFootprintPoint[],
): Array<[number, number]> | null {
  const coordinates = footprint.map((point) => localToMercator(anchor, transformFootprintPoint(pose, point)));
  if (coordinates.some((coordinate) => coordinate === null)) {
    return null;
  }
  return coordinates as Array<[number, number]>;
}

function footprintFrontEdgeToMercator(
  anchor: ProjectionAnchor,
  pose: Transform2D,
  footprint: MowerFootprintPoint[],
): Array<[number, number]> | null {
  if (footprint.length < 2) {
    return null;
  }
  const maxX = Math.max(...footprint.map((point) => point[0]));
  const frontPoints = footprint
    .filter((point) => Math.abs(point[0] - maxX) < 0.000001)
    .sort((a, b) => b[1] - a[1]);
  if (frontPoints.length < 2) {
    return null;
  }
  const edge = [
    localToMercator(anchor, transformFootprintPoint(pose, frontPoints[0])),
    localToMercator(anchor, transformFootprintPoint(pose, frontPoints[frontPoints.length - 1])),
  ];
  if (edge.some((coordinate) => coordinate === null)) {
    return null;
  }
  return edge as Array<[number, number]>;
}

function distance2D(a: Pick<MapPoint, "x" | "y"> | null, b: Pick<MapPoint, "x" | "y"> | null): number {
  if (!a || !b) {
    return Number.NaN;
  }
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function distanceToMapBounds(summary: MapSummary | null, pose: Pick<MapPoint, "x" | "y"> | null): number {
  if (!summary?.bounds_valid || !pose) {
    return Number.NaN;
  }
  const dx = pose.x < summary.min_x ? summary.min_x - pose.x : pose.x > summary.max_x ? pose.x - summary.max_x : 0;
  const dy = pose.y < summary.min_y ? summary.min_y - pose.y : pose.y > summary.max_y ? pose.y - summary.max_y : 0;
  return Math.hypot(dx, dy);
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

function openMapRing(points: MapPoint[]): MapPoint[] {
  if (points.length < 2) {
    return points;
  }
  const first = points[0];
  const last = points[points.length - 1];
  if (first && last && Math.abs(first.x - last.x) < 0.000001 && Math.abs(first.y - last.y) < 0.000001) {
    return points.slice(0, -1);
  }
  return points;
}

function footprintCenterOffsetX(footprint: MowerFootprintPoint[]): number {
  const xs = footprint.map((point) => point[0]).filter((value) => Number.isFinite(value));
  if (xs.length === 0) {
    return 0;
  }
  return (Math.min(...xs) + Math.max(...xs)) * 0.5;
}

function routePoseTransform(pose: RoutePlanPose): Transform2D {
  return {
    x: pose.x,
    y: pose.y,
    yaw: pose.yaw,
  };
}

function routePoseToolPoint(pose: RoutePlanPose, offsetM: number): MapPoint {
  if (Number.isFinite(pose.tool_x) && Number.isFinite(pose.tool_y)) {
    return { x: Number(pose.tool_x), y: Number(pose.tool_y) };
  }
  return {
    x: pose.x + offsetM * Math.cos(pose.yaw),
    y: pose.y + offsetM * Math.sin(pose.yaw),
  };
}

function routePoseToolYaw(pose: RoutePlanPose): number {
  return Number.isFinite(pose.tool_yaw) ? Number(pose.tool_yaw) : pose.yaw;
}

function routePathBasePoints(path: RoutePlanPath): MapPoint[] {
  return path.poses.map((pose) => ({ x: pose.x, y: pose.y }));
}

function routePathToolPoints(path: RoutePlanPath, offsetM: number): MapPoint[] {
  return path.poses.map((pose) => routePoseToolPoint(pose, offsetM));
}

function routePathRemainingToolPoints(routePlan: RoutePlan, path: RoutePlanPath, offsetM: number): MapPoint[] {
  if (path.poses.length === 0) {
    return [];
  }
  if (path.path_index < routePlan.current_path_index) {
    return [];
  }
  const startIndex =
    path.path_index === routePlan.current_path_index ? Math.max(0, Math.min(path.poses.length - 1, routePlan.current_pose_index)) : 0;
  return path.poses.slice(startIndex).map((pose) => routePoseToolPoint(pose, offsetM));
}

function navPathPoints(path: NavPath | null): MapPoint[] {
  if (!path?.poses?.length) {
    return [];
  }
  return path.poses
    .map((pose) => ({
      x: Number(pose.pose?.position?.x),
      y: Number(pose.pose?.position?.y),
    }))
    .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
}

function rosPolygonFromPoints(points: MapPoint[]): { points: Array<{ x: number; y: number; z: number }> } {
  return {
    points: points.map((point) => ({ x: point.x, y: point.y, z: 0 })),
  };
}

function nearestVertexIndex(points: MapPoint[], target: MapPoint, maxDistanceM = 0.8): number {
  let bestIndex = -1;
  let bestDistance = maxDistanceM;
  points.forEach((point, index) => {
    const distance = Math.hypot(point.x - target.x, point.y - target.y);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = index;
    }
  });
  return bestIndex;
}

function nearestSegmentInsertIndex(points: MapPoint[], target: MapPoint, maxDistanceM = 1.0): number {
  if (points.length < 2) {
    return -1;
  }
  let bestIndex = -1;
  let bestDistance = maxDistanceM;
  for (let index = 0; index < points.length; index += 1) {
    const a = points[index];
    const b = points[(index + 1) % points.length];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const lengthSq = dx * dx + dy * dy;
    const t = lengthSq <= 0 ? 0 : Math.max(0, Math.min(1, ((target.x - a.x) * dx + (target.y - a.y) * dy) / lengthSq));
    const px = a.x + dx * t;
    const py = a.y + dy * t;
    const distance = Math.hypot(target.x - px, target.y - py);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = index + 1;
    }
  }
  return bestIndex;
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
  if (kind === "finalFootprint") {
    const confidence = Math.max(0, Math.min(1, Number(feature.get("confidence") ?? 0)));
    return new Style({
      fill: new Fill({ color: `rgba(255, 255, 255, ${0.14 + confidence * 0.16})` }),
      stroke: new Stroke({ color: "rgba(23, 32, 42, 0.92)", width: 2.5 }),
      zIndex: 1000,
    });
  }
  if (kind === "finalFootprintFront") {
    return new Style({
      stroke: new Stroke({ color: "#2459a6", width: 4 }),
      zIndex: 1001,
    });
  }

  const isLidar = kind === "lidarRobot";
  const palette = isLidar
    ? {
        color: "#5b4bb7",
        halo: "91, 75, 183",
        label: "LIDAR",
        zHalo: 40,
        zMarker: 42,
      }
    : {
        color: "#0f7a7a",
        halo: "15, 122, 122",
        label: "GPS",
        zHalo: 39,
        zMarker: 41,
      };
  const yaw = Number(feature.get("yaw") ?? 0);
  const showLabel = Boolean(feature.get("showLabel"));
  const confidence = Math.max(0, Math.min(1, Number(feature.get("confidence") ?? 0)));
  return [
    new Style({
      image: new CircleStyle({
        fill: new Fill({ color: `rgba(${palette.halo}, ${0.08 + confidence * 0.16})` }),
        radius: 20 + confidence * 7,
        stroke: new Stroke({ color: `rgba(${palette.halo}, 0.22)`, width: 1.5 }),
      }),
      zIndex: palette.zHalo,
    }),
    new Style({
      image: new RegularShape({
        fill: new Fill({ color: palette.color }),
        points: 3,
        radius: 14,
        rotation: Math.PI / 2 - yaw,
        stroke: new Stroke({ color: "#ffffff", width: 2.5 }),
      }),
      text: showLabel
        ? new Text({
            fill: new Fill({ color: palette.color }),
            font: "800 12px Inter, system-ui, sans-serif",
            offsetY: -25,
            stroke: new Stroke({ color: "rgba(255,255,255,0.94)", width: 4 }),
            text: palette.label,
          })
        : undefined,
      zIndex: palette.zMarker,
    }),
  ];
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

function confidenceClass(value: number | null | undefined): string {
  if (value === null || value === undefined || !Number.isFinite(value)) {
    return "unknown";
  }
  if (value >= 0.7) {
    return "good";
  }
  if (value >= 0.35) {
    return "degraded";
  }
  return "poor";
}

function componentSummary(components?: Record<string, number>): string {
  if (!components) {
    return "--";
  }
  const entries = Object.entries(components).filter(([, value]) => Number.isFinite(value));
  if (entries.length === 0) {
    return "--";
  }
  return entries.map(([key, value]) => `${key.replace(/_/g, " ")} ${Math.round(value * 100)}%`).join(", ");
}

function reasonSummary(reasons?: string[]): string {
  return reasons && reasons.length > 0 ? reasons.map((reason) => reason.replace(/_/g, " ")).join(", ") : "none";
}

function fusionWeight(status: LocalizationFusionStatus | null, source: string): number | null {
  const fromAccepted = status?.accepted_source_weights?.[source];
  if (Number.isFinite(fromAccepted)) {
    return Number(fromAccepted);
  }
  const fromWeights = status?.source_weights?.[source];
  if (Number.isFinite(fromWeights)) {
    return Number(fromWeights);
  }
  const fromSource = status?.sources?.[source]?.weight;
  return Number.isFinite(fromSource) ? Number(fromSource) : null;
}

function statusLabel(value: string | null | undefined): string {
  return value ? value.replace(/_/g, " ") : "--";
}

function overlayStrokeColor(colorName: string): string {
  switch (colorName.toLowerCase()) {
    case "green":
      return "#168a66";
    case "red":
      return "#b42318";
    case "blue":
      return "#2459a6";
    default:
      return "#17202a";
  }
}

function recordingOverlayFeatureStyle(feature: FeatureLike): Style {
  const kind = String(feature.get("kind") ?? "");
  const color = overlayStrokeColor(String(feature.get("overlayColor") ?? "blue"));
  const closed = Boolean(feature.get("closed"));
  const width = Math.max(2, Math.min(7, Number(feature.get("lineWidth") ?? 0.08) * 36));

  if (kind === "planToolCenter") {
    return new Style({
      stroke: new Stroke({
        color,
        width: Math.max(3, width + 1),
      }),
      zIndex: 760,
    });
  }

  if (kind === "planBaseLink") {
    return new Style({
      stroke: new Stroke({
        color: `${color}99`,
        lineDash: [3, 6],
        width: Math.max(1.5, width * 0.45),
      }),
      zIndex: 720,
    });
  }

  return new Style({
    fill: closed ? new Fill({ color: `${color}24` }) : undefined,
    stroke: new Stroke({
      color,
      lineDash: closed ? undefined : [8, 6],
      width,
    }),
  });
}

function actualTrackFeatureStyle(feature: FeatureLike): Style {
  const kind = String(feature.get("kind") ?? "");
  if (kind === "actualTrackHead") {
    return new Style({
      image: new CircleStyle({
        fill: new Fill({ color: "#ffffff" }),
        radius: 5,
        stroke: new Stroke({ color: "#d97706", width: 2 }),
      }),
      zIndex: 950,
    });
  }

  return new Style({
    stroke: new Stroke({
      color: "rgba(217, 119, 6, 0.92)",
      lineCap: "round",
      lineJoin: "round",
      width: 3,
    }),
    zIndex: 900,
  });
}

function routeFeatureStyle(feature: FeatureLike): Style {
  const kind = String(feature.get("kind") ?? "");
  if (kind === "planRouteBase") {
    return new Style({
      stroke: new Stroke({ color: "rgba(36, 89, 166, 0.68)", lineDash: [6, 7], width: 2.5 }),
      zIndex: 735,
    });
  }
  if (kind === "planRouteRemaining") {
    return new Style({
      stroke: new Stroke({ color: "rgba(20, 83, 45, 0.94)", lineCap: "round", lineJoin: "round", width: 4 }),
      zIndex: 745,
    });
  }
  if (kind === "routeBase") {
    return new Style({
      stroke: new Stroke({ color: "rgba(36, 89, 166, 0.82)", lineDash: [7, 5], width: 3 }),
      zIndex: 820,
    });
  }
  if (kind === "routeTool") {
    return new Style({
      stroke: new Stroke({ color: "rgba(20, 83, 45, 0.96)", lineCap: "round", lineJoin: "round", width: 4 }),
      zIndex: 840,
    });
  }
  if (kind === "routeOutlineFootprint") {
    return new Style({
      fill: new Fill({ color: "rgba(20, 83, 45, 0.035)" }),
      stroke: new Stroke({ color: "rgba(20, 83, 45, 0.16)", width: 1 }),
      zIndex: 780,
    });
  }
  if (kind === "routeSelectedFootprint") {
    return new Style({
      fill: new Fill({ color: "rgba(217, 119, 6, 0.16)" }),
      stroke: new Stroke({ color: "rgba(217, 119, 6, 0.95)", width: 2.5 }),
      zIndex: 900,
    });
  }
  return new Style({
    image: new CircleStyle({
      fill: new Fill({ color: "#ffffff" }),
      radius: 5,
      stroke: new Stroke({ color: "#14532d", width: 2 }),
    }),
    zIndex: 920,
  });
}

function mbfPathFeatureStyle(feature: FeatureLike): Style {
  const kind = String(feature.get("kind") ?? "");
  if (kind === "mbfGlobalPlan") {
    return new Style({
      stroke: new Stroke({ color: "rgba(185, 28, 28, 0.86)", lineDash: [10, 6], width: 3 }),
      zIndex: 810,
    });
  }

  return new Style({
    stroke: new Stroke({ color: "rgba(194, 65, 12, 0.9)", lineCap: "round", lineJoin: "round", width: 3 }),
    zIndex: 815,
  });
}

function editorFeatureStyle(feature: FeatureLike): Style | Style[] {
  const kind = String(feature.get("kind") ?? "");
  if (kind === "editorVertex") {
    return new Style({
      image: new CircleStyle({
        fill: new Fill({ color: "#ffffff" }),
        radius: 5,
        stroke: new Stroke({ color: "#2459a6", width: 2 }),
      }),
      zIndex: 1200,
    });
  }
  if (kind === "editorStroke") {
    const operation = String(feature.get("operation") ?? "pen");
    return new Style({
      stroke: new Stroke({
        color: operation === "eraser" ? "rgba(180, 35, 24, 0.95)" : "rgba(15, 122, 122, 0.95)",
        lineCap: "round",
        lineJoin: "round",
        width: Math.max(5, Math.min(22, Number(feature.get("brushDiameterM") ?? 0.7) * 16)),
      }),
      zIndex: 1100,
    });
  }
  return new Style({
    fill: new Fill({ color: "rgba(36, 89, 166, 0.12)" }),
    stroke: new Stroke({ color: "rgba(36, 89, 166, 0.95)", lineDash: [7, 5], width: 2.5 }),
    zIndex: 1090,
  });
}

function clampUnit(value: number): number {
  return Math.max(-1, Math.min(1, value));
}

interface AreaJoystickPadProps {
  command: JoystickCommand;
  disabled: boolean;
  onCommand: (command: JoystickCommand) => void;
  onRelease: () => void;
}

function AreaJoystickPad({ command, disabled, onCommand, onRelease }: AreaJoystickPadProps) {
  const activePointerIdRef = useRef<number | null>(null);
  const knobX = clampUnit(-command.z / 1.6) * 44;
  const knobY = clampUnit(-command.x) * 44;

  function updateFromPointer(event: ReactPointerEvent<HTMLDivElement>): void {
    if (disabled || activePointerIdRef.current !== event.pointerId) {
      return;
    }
    const rect = event.currentTarget.getBoundingClientRect();
    const radius = Math.max(1, Math.min(rect.width, rect.height) / 2);
    const x = clampUnit((event.clientX - rect.left - rect.width / 2) / radius);
    const y = clampUnit((event.clientY - rect.top - rect.height / 2) / radius);
    onCommand({ x: -y, z: -x * 1.6 });
  }

  function release(event: ReactPointerEvent<HTMLDivElement>): void {
    if (activePointerIdRef.current !== event.pointerId) {
      return;
    }
    activePointerIdRef.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    onRelease();
  }

  return (
    <div
      className={`area-joystick-pad ${disabled ? "is-disabled" : ""}`}
      onPointerCancel={release}
      onPointerDown={(event) => {
        if (disabled) {
          return;
        }
        event.preventDefault();
        activePointerIdRef.current = event.pointerId;
        event.currentTarget.setPointerCapture(event.pointerId);
        updateFromPointer(event);
      }}
      onPointerLeave={(event) => {
        if (activePointerIdRef.current !== event.pointerId || !event.currentTarget.hasPointerCapture(event.pointerId)) {
          return;
        }
        updateFromPointer(event);
      }}
      onPointerMove={updateFromPointer}
      onPointerUp={release}
      role="presentation"
    >
      <div className="area-joystick-axis area-joystick-axis-x" />
      <div className="area-joystick-axis area-joystick-axis-y" />
      <div className="area-joystick-knob" style={{ transform: `translate(${knobX}px, ${knobY}px)` }} />
    </div>
  );
}

export function CombinedMapView({ config, connected, error, mowerInputStatus, now, routeMode = false, ros, url }: CombinedMapViewProps) {
  const [layers, setLayers] = useState<LayerSettings>(() => ({
    actualTrack: config.defaultLayerActualTrack,
    calibration: config.defaultLayerCalibration,
    gpsAccuracy: config.defaultLayerGpsAccuracy,
    labels: config.defaultLayerLabels,
    liveScan: config.defaultLayerLiveScan,
    mbfPath: config.defaultLayerMbfPath,
    mowerMap: config.defaultLayerMowerMap,
    planProgress: config.defaultLayerPlanProgress,
    satellite: config.defaultLayerSatellite,
    slamObstacles: config.defaultLayerSlamObstacles,
  }));
  const [routeLayers, setRouteLayers] = useState<RouteLayerSettings>(() => ({
    actualTrack: true,
    basePath: true,
    mowerMap: config.defaultLayerMowerMap,
    outlineFootprints: false,
    satellite: config.defaultLayerSatellite,
    selectedFootprint: true,
    toolPath: true,
  }));
  const [routeSourceMode, setRouteSourceMode] = useState<RouteSourceMode>("live");
  const [importedRoutePlan, setImportedRoutePlan] = useState<RoutePlan | null>(null);
  const [routeImportMessage, setRouteImportMessage] = useState<string | null>(null);
  const [routePreviewMessage, setRoutePreviewMessage] = useState<string | null>(null);
  const [routePreviewPending, setRoutePreviewPending] = useState(false);
  const [selectedRoutePathListIndex, setSelectedRoutePathListIndex] = useState(0);
  const [selectedRoutePoseIndex, setSelectedRoutePoseIndex] = useState(0);
  const [projectionAnchor, setProjectionAnchor] = useState<ProjectionAnchor | null>(null);
  const [commandMessage, setCommandMessage] = useState<string | null>(null);
  const [commandPending, setCommandPending] = useState(false);
  const [warningOpen, setWarningOpen] = useState(false);
  const [mapOpacity, setMapOpacity] = useState(0.42);
  const [scanClampMeters, setScanClampMeters] = useState(12);
  const [paused, setPaused] = useState(false);
  const [mapResetKey, setMapResetKey] = useState(0);
  const [areaCommandMessage, setAreaCommandMessage] = useState<string | null>(null);
  const [areaRecordingUseFusedPose, setAreaRecordingUseFusedPose] = useState(true);
  const [areaRecordingPoseSourcePending, setAreaRecordingPoseSourcePending] = useState(false);
  const [manualPathPendingAction, setManualPathPendingAction] = useState<string | null>(null);
  const [manualPathRawBag, setManualPathRawBag] = useState(false);
  const [finishAreaDialogOpen, setFinishAreaDialogOpen] = useState(false);
  const [mapCommandMessage, setMapCommandMessage] = useState<string | null>(null);
  const [mapCommandPending, setMapCommandPending] = useState(false);
  const [newMapName, setNewMapName] = useState("");
  const [mapNameDialog, setMapNameDialog] = useState<"create" | "rename" | null>(null);
  const [mapSelectorOpen, setMapSelectorOpen] = useState(false);
  const [renameMapName, setRenameMapName] = useState("");
  const [editorMode, setEditorMode] = useState<EditorMode>("view");
  const [paintTool, setPaintTool] = useState<PaintTool>("pen");
  const [brushSizeKey, setBrushSizeKey] = useState<BrushSizeKey>("mower");
  const [polygonTool, setPolygonTool] = useState<PolygonTool>("move");
  const [selectedEditAreaId, setSelectedEditAreaId] = useState("");
  const [pendingStroke, setPendingStroke] = useState<MapPoint[]>([]);
  const [draftPolygon, setDraftPolygon] = useState<MapPoint[] | null>(null);
  const [editCommandMessage, setEditCommandMessage] = useState<string | null>(null);
  const [editCommandPending, setEditCommandPending] = useState(false);

  const actionTopicRef = useRef<Topic<StringMessage> | null>(null);
  const mapElementRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<OlMap | null>(null);
  const mapSelectorRef = useRef<HTMLDivElement | null>(null);
  const newMapNameInputRef = useRef<HTMLInputElement | null>(null);
  const renameMapNameInputRef = useRef<HTMLInputElement | null>(null);
  const editorDrawingRef = useRef(false);
  const editorDragVertexRef = useRef<number | null>(null);
  const editorActivePointerIdRef = useRef<number | null>(null);
  const editorDisabledDragPanRef = useRef<Array<{ interaction: DragPan; wasActive: boolean }>>([]);
  const draftPolygonRef = useRef<MapPoint[] | null>(null);
  const hasCenteredOnFirstFixRef = useRef(false);
  const actualTrackPointsRef = useRef<MapPoint[]>([]);
  const actualTrackWasMowingRef = useRef(false);
  const actualTrackSourceRef = useRef<VectorSource | null>(null);
  const editorSourceRef = useRef<VectorSource | null>(null);
  const mbfPathSourceRef = useRef<VectorSource | null>(null);
  const mowerSourceRef = useRef<VectorSource | null>(null);
  const positionSourceRef = useRef<VectorSource | null>(null);
  const calibrationSourceRef = useRef<VectorSource | null>(null);
  const recordingOverlaySourceRef = useRef<VectorSource | null>(null);
  const routeSourceRef = useRef<VectorSource | null>(null);
  const scanSourceRef = useRef<VectorSource | null>(null);
  const slamSourceRef = useRef<ImageCanvasSource | null>(null);
  const satelliteLayerRef = useRef<BaseLayer | null>(null);
  const slamLayerRef = useRef<ImageLayer<ImageCanvasSource> | null>(null);
  const calibrationLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const actualTrackLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const editorLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const mbfPathLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const mowerLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const recordingOverlayLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
  const routeLayerRef = useRef<VectorLayer<VectorSource> | null>(null);
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
  const { stats: unifiedPoseStats, status: unifiedPose } = useGpsStatus({
    ros,
    topicName: config.localizationFusionPoseTopic,
  });
  const { mapData, stats: mowerMapStats } = useMowerMap({
    ros,
    topicName: config.mowerMapTopic,
  });
  const { catalog: mapCatalog, selectedMap, stats: mapCatalogStats } = useMapCatalog({
    ros,
    topicName: config.mapCatalogTopic,
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
  const { stats: tfStats, transformHistory, transforms } = useTfFrames({
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
  const { stats: confidenceStats, status: confidenceStatus } = useLocalizationConfidenceStatus({
    ros,
    topicName: config.localizationConfidenceTopic,
  });
  const { stats: fusionStats, status: fusionStatus } = useLocalizationFusionStatus({
    ros,
    topicName: config.localizationFusionStatusTopic,
  });
  const { actions: mowerActions, enabledActionIds, stats: actionsStats } = useAvailableActions({
    ros,
    topicName: config.actionsTopic,
  });
  const { overlay: recordingOverlay, stats: recordingOverlayStats } = useMapOverlay({
    ros,
    topicName: config.mapOverlayTopic,
  });
  const toolCenterOffsetM = useMemo(() => footprintCenterOffsetX(config.mowerFootprint), [config.mowerFootprint]);
  const { robotState, stats: robotStateStats } = useRobotState({
    ros,
    topicName: config.robotStateTopic,
  });
  const { routePlan: liveRoutePlan, stats: routePlanStats } = useRoutePlan({
    ros,
    topicName: config.routePlanTopic,
  });
  const { path: mbfGlobalPlan } = useNavPath({
    ros,
    topicName: config.mbfGlobalPlanTopic,
  });
  const { path: mbfControllerPlan } = useNavPath({
    ros,
    topicName: config.mbfControllerPlanTopic,
  });
  const { stats: manualPathRecorderStats, status: manualPathRecorderStatus } = useManualPathRecorderStatus({
    ros,
    topicName: config.manualPathRecorderStatusTopic,
  });

  useEffect(() => {
    if (!ros) {
      actionTopicRef.current = null;
      return undefined;
    }

    const topic = new Topic<StringMessage>({
      messageType: "std_msgs/String",
      name: config.actionTopic,
      queue_length: 1,
      ros,
    });
    topic.advertise();
    actionTopicRef.current = topic;

    return () => {
      topic.unadvertise();
      if (actionTopicRef.current === topic) {
        actionTopicRef.current = null;
      }
    };
  }, [config.actionTopic, ros]);

  useEffect(() => {
    if (!ros) {
      return undefined;
    }

    const topic = new Topic<BoolMessage>({
      messageType: "std_msgs/Bool",
      name: config.areaRecordingUseFusedPoseTopic,
      queue_length: 1,
      ros,
    });
    topic.subscribe((message) => setAreaRecordingUseFusedPose(Boolean(message.data)));

    return () => {
      topic.unsubscribe();
    };
  }, [config.areaRecordingUseFusedPoseTopic, ros]);

  useEffect(() => {
    setRenameMapName(selectedMap?.name ?? "");
  }, [selectedMap?.id, selectedMap?.name]);

  useEffect(() => {
    if (!mapSelectorOpen) {
      return undefined;
    }

    function closeFromOutside(event: PointerEvent): void {
      if (mapNameDialog) {
        return;
      }
      const target = event.target;
      if (target instanceof Node && mapSelectorRef.current?.contains(target)) {
        return;
      }
      setMapSelectorOpen(false);
    }

    function closeFromEscape(event: KeyboardEvent): void {
      if (event.key === "Escape") {
        setMapSelectorOpen(false);
        setMapNameDialog(null);
      }
    }

    document.addEventListener("pointerdown", closeFromOutside);
    document.addEventListener("keydown", closeFromEscape);
    return () => {
      document.removeEventListener("pointerdown", closeFromOutside);
      document.removeEventListener("keydown", closeFromEscape);
    };
  }, [mapNameDialog, mapSelectorOpen]);

  const publishMowerAction = useCallback(
    (action: string, source = "ui") => {
      if (!connected || !actionTopicRef.current) {
        setAreaCommandMessage("ROS action channel unavailable");
        return;
      }
      actionTopicRef.current.publish({ data: action });
      const actionInfo = mowerActions.find((candidate) => candidate.action_id === action);
      setAreaCommandMessage(actionInfo?.action_name || action.replace(/^mower_logic:/, ""));
      if (source !== "ui") {
        window.setTimeout(() => setAreaCommandMessage(null), 1400);
      }
    },
    [connected, mowerActions],
  );

  const currentState = robotState?.current_state || "Unknown";
  const currentSubState = robotState?.current_sub_state || "";
  const areaRecordingActive = currentState === "AREA_RECORDING";
  const mowingOverlayActive = currentState === "MOWING" || currentState === "PAUSED";
  const manualInputSource = mowerInputStatus?.active_source ?? "web_gamepad";
  const webManualInputActive = manualInputSource === "web_gamepad";
  const directManualInputActive = manualInputSource === "direct_bluetooth";
  const directManualInputConnected = Boolean(mowerInputStatus?.direct_connected);
  const mapMutationAllowed = connected && currentState === "IDLE";
  const recordingInProgress = enabledActionIds.has(ACTION_STOP_RECORDING);
  const recordingEditAllowed = connected && areaRecordingActive && !recordingInProgress;
  const savedMapEditAllowed = mapMutationAllowed && Boolean(selectedMap);
  const editMutationAllowed = savedMapEditAllowed || recordingEditAllowed;
  const hasEnabledAction = useCallback((action: string) => enabledActionIds.has(action), [enabledActionIds]);
  const hasAnyFinishAction = hasEnabledAction(ACTION_FINISH_MOWING_AREA) || hasEnabledAction(ACTION_FINISH_NAVIGATION_AREA) || hasEnabledAction(ACTION_FINISH_DISCARD);
  const activeRoutePlan = routeSourceMode === "import" ? importedRoutePlan : liveRoutePlan;
  const activeRouteKey = activeRoutePlan
    ? `${routeSourceMode}:${activeRoutePlan.source}:${activeRoutePlan.plan_id}:${activeRoutePlan.paths.length}`
    : `${routeSourceMode}:none`;
  const selectedRoutePath = activeRoutePlan?.paths[selectedRoutePathListIndex] ?? null;
  const selectedRoutePose = selectedRoutePath?.poses[selectedRoutePoseIndex] ?? null;
  const selectedRoutePoseCount = selectedRoutePath?.poses.length ?? 0;
  const selectedRouteIsOutline = Boolean(selectedRoutePath?.is_outline);
  const routeImportActive = routeSourceMode === "import" && Boolean(importedRoutePlan);
  const routeStatus = activeRoutePlan
    ? `${activeRoutePlan.source.replace(/_/g, " ")}${activeRoutePlan.active ? " active" : " inactive"}`
    : routeSourceMode === "import"
      ? "No imported route"
      : "No live route";
  const satelliteLayerVisible = routeMode ? routeLayers.satellite : layers.satellite;
  const mowerMapLayerVisible = routeMode ? routeLayers.mowerMap : layers.mowerMap;
  const actualTrackLayerVisible = routeMode ? routeLayers.actualTrack : layers.actualTrack;
  const recordingPoseSourceLabel = areaRecordingUseFusedPose ? "Final Pose" : "GPS Pose";
  const manualPathActive = Boolean(manualPathRecorderStatus?.active);
  const manualPathOnline = manualPathRecorderStats.lastMessageAt !== null && now - manualPathRecorderStats.lastMessageAt <= 3000;
  const manualPathSessionAvailable = Boolean(manualPathRecorderStatus?.session_id);
  const manualPathSyncSamples = manualPathRecorderStatus?.sample_counts?.sync_sample ?? 0;
  const manualPathRejectCount = Object.values(manualPathRecorderStatus?.reject_counts ?? {}).reduce((sum, count) => sum + Number(count || 0), 0);
  const manualPathStatusLabel = manualPathActive
    ? manualPathRecorderStatus?.raw_bag_active
      ? "Capturing · Bag"
      : "Capturing"
    : manualPathOnline
      ? manualPathSessionAvailable
        ? "Ready"
        : "Idle"
      : "Offline";

  async function setAreaRecordingPoseSource(nextEnabled: boolean): Promise<void> {
    if (!ros || !connected || areaRecordingPoseSourcePending || recordingInProgress) {
      return;
    }

    setAreaRecordingPoseSourcePending(true);
    setAreaCommandMessage(null);
    try {
      const response = await callSetBoolService(ros, config.areaRecordingUseFusedPoseService, nextEnabled);
      setAreaCommandMessage(response.message || (nextEnabled ? "Final pose recording" : "GPS pose recording"));
      if (!response.success) {
        throw new Error(response.message || "Pose source change failed");
      }
    } catch (serviceError) {
      setAreaCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setAreaRecordingPoseSourcePending(false);
    }
  }

  async function startManualPathCapture(): Promise<void> {
    if (!ros || !connected || !areaRecordingActive || manualPathActive || manualPathPendingAction) {
      return;
    }

    setManualPathPendingAction("start");
    setAreaCommandMessage(null);
    try {
      const response = await callSetBoolService(ros, config.manualPathRecorderStartService, manualPathRawBag);
      setAreaCommandMessage(response.message || "Path capture started");
      if (!response.success) {
        throw new Error(response.message || "Path capture start failed");
      }
    } catch (serviceError) {
      setAreaCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setManualPathPendingAction(null);
    }
  }

  async function callManualPathRecorderAction(action: "export" | "mark" | "stop"): Promise<void> {
    if (!ros || !connected || manualPathPendingAction) {
      return;
    }

    const serviceName =
      action === "stop"
        ? config.manualPathRecorderStopService
        : action === "mark"
          ? config.manualPathRecorderMarkEventService
          : config.manualPathRecorderExportService;
    setManualPathPendingAction(action);
    setAreaCommandMessage(null);
    try {
      const response = await callTriggerService(ros, serviceName);
      setAreaCommandMessage(response.message || "Path capture updated");
      if (!response.success) {
        throw new Error(response.message || "Path capture command failed");
      }
    } catch (serviceError) {
      setAreaCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setManualPathPendingAction(null);
    }
  }

  useEffect(() => {
    if (!activeRoutePlan || activeRoutePlan.paths.length === 0) {
      setSelectedRoutePathListIndex(0);
      setSelectedRoutePoseIndex(0);
      return;
    }
    const currentPathIndex = activeRoutePlan.paths.findIndex((path) => path.path_index === activeRoutePlan.current_path_index);
    const nextPathListIndex = currentPathIndex >= 0 ? currentPathIndex : 0;
    const nextPath = activeRoutePlan.paths[nextPathListIndex];
    setSelectedRoutePathListIndex(nextPathListIndex);
    setSelectedRoutePoseIndex(Math.max(0, Math.min(nextPath.poses.length - 1, activeRoutePlan.current_pose_index)));
  }, [activeRouteKey]);

  useEffect(() => {
    if (!selectedRoutePath || selectedRoutePoseIndex < selectedRoutePath.poses.length) {
      return;
    }
    setSelectedRoutePoseIndex(Math.max(0, selectedRoutePath.poses.length - 1));
  }, [selectedRoutePath, selectedRoutePoseIndex]);
  const areaControl = useAreaRecordingControl({
    active: areaRecordingActive && webManualInputActive,
    connected,
    joyTopic: config.webJoyTopic,
    publishAction: publishMowerAction,
    ros,
  });
  const editableAreas = useMemo(
    () =>
      (mapData?.areas ?? []).filter(
        (area) => area.properties?.active !== false && ["mow", "nav", "obstacle"].includes(mowerAreaType(area)) && area.outline.length >= 3,
      ),
    [mapData?.areas],
  );
  const selectedEditArea = useMemo(() => {
    if (!editableAreas.length) {
      return null;
    }
    return editableAreas.find((area) => area.id === selectedEditAreaId) ?? editableAreas.find((area) => mowerAreaType(area) === "mow") ?? editableAreas[0];
  }, [editableAreas, selectedEditAreaId]);
  const brushDiameter = BRUSH_SIZES[brushSizeKey].value;

  useEffect(() => {
    if (!editableAreas.length) {
      setSelectedEditAreaId("");
      return;
    }
    const current = editableAreas.find((area) => area.id === selectedEditAreaId);
    if (!current) {
      setSelectedEditAreaId((editableAreas.find((area) => mowerAreaType(area) === "mow") ?? editableAreas[0]).id);
    }
  }, [editableAreas, selectedEditAreaId]);

  useEffect(() => {
    if (editorMode !== "polygon" || !selectedEditArea) {
      setDraftPolygon(null);
      return;
    }
    setDraftPolygon(openMapRing(selectedEditArea.outline));
  }, [editorMode, selectedEditArea?.id, selectedMap?.map_hash]);

  useEffect(() => {
    draftPolygonRef.current = draftPolygon;
  }, [draftPolygon]);

  useEffect(() => {
    if (areaRecordingActive && editorMode === "polygon") {
      setEditorMode("paint");
    }
  }, [areaRecordingActive, editorMode]);

  const alignmentIsFresh = alignmentStats.lastMessageAt !== null && now - alignmentStats.lastMessageAt <= GPS_STALE_MS;
  const confidenceIsFresh = confidenceStats.lastMessageAt !== null && now - confidenceStats.lastMessageAt <= GPS_STALE_MS;
  const fusionStatusIsFresh = fusionStats.lastMessageAt !== null && now - fusionStats.lastMessageAt <= GPS_STALE_MS;
  const freshFusionStatus = fusionStatusIsFresh ? fusionStatus : null;
  const gpsConfidence = confidenceIsFresh ? confidenceStatus?.gps.position_confidence : null;
  const lidarLocalConfidence = confidenceIsFresh ? confidenceStatus?.lidar.local_confidence : null;
  const lidarGlobalConfidence = confidenceIsFresh ? confidenceStatus?.lidar.global_confidence : null;
  const unifiedConfidence = freshFusionStatus?.confidence;
  const boundarySkipCount = Object.values(alignmentStatus?.boundary_sample_skip_counts ?? {}).reduce(
    (total, value) => total + (Number.isFinite(value) ? Number(value) : 0),
    0,
  );
  const statusMapToSlam = alignmentIsFresh ? poseFromAlignmentPose(alignmentStatus?.transform) : null;
  const statusGpsPose = alignmentIsFresh ? poseFromAlignmentPose(alignmentStatus?.gps_pose) : null;
  const statusLidarPose = alignmentIsFresh ? poseFromAlignmentPose(alignmentStatus?.lidar_pose) : null;
  const tfLookupOptions = useMemo(() => ({ maxAgeMs: TF_STALE_MS, nowMs: now }), [now]);
  const mapToOperationalBase = useMemo(
    () => lookupTransform2D(transforms, "map", "base_link", tfLookupOptions),
    [tfLookupOptions, transforms],
  );
  const mapToUnifiedBaseTf = useMemo(
    () => lookupTransform2D(transforms, "map", config.localizationFusionBaseFrame, tfLookupOptions),
    [config.localizationFusionBaseFrame, tfLookupOptions, transforms],
  );
  const fusedPoseTransform = useMemo(() => poseFromAbsolutePose(fusedPose), [fusedPose]);
  const freshFusedPose =
    fusedPoseStats.lastMessageAt !== null && now - fusedPoseStats.lastMessageAt <= GPS_STALE_MS
      ? fusedPoseTransform
      : null;
  const unifiedPoseTransform = useMemo(() => poseFromAbsolutePose(unifiedPose), [unifiedPose]);
  const freshUnifiedPose =
    unifiedPoseStats.lastMessageAt !== null && now - unifiedPoseStats.lastMessageAt <= GPS_STALE_MS
      ? unifiedPoseTransform
      : null;
  const unifiedMapPose = freshUnifiedPose ?? mapToUnifiedBaseTf;
  const gpsPose = statusGpsPose ?? mapToOperationalBase ?? freshFusedPose;
  const actualTrackPose = unifiedMapPose ?? gpsPose;
  const selectedMapDistance = distanceToMapBounds(selectedMap, unifiedMapPose ?? gpsPose);
  const selectedMapFar = Number.isFinite(selectedMapDistance) && selectedMapDistance > 50;
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
  const scanStampMs = rosStampMs(scan?.header.stamp);
  const tfScanToMap = useMemo(
    () => lookupTransform2D(transforms, "map", scanFrame, tfLookupOptions),
    [scanFrame, tfLookupOptions, transforms],
  );
  const timestampedScanToMap = useMemo(
    () =>
      scanStampMs === null
        ? null
        : lookupTransform2DAt(transforms, transformHistory, "map", scanFrame, scanStampMs, { maxStampDeltaMs: 450 }),
    [scanFrame, scanStampMs, transformHistory, transforms],
  );
  const slamToScan = useMemo(
    () => lookupTransform2D(transforms, config.slamMapFrame, scanFrame, tfLookupOptions),
    [config.slamMapFrame, scanFrame, tfLookupOptions, transforms],
  );
  const scanToMap = timestampedScanToMap ?? tfScanToMap ?? (statusMapToSlam && slamToScan ? composeTransform(statusMapToSlam, slamToScan) : null);

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
  const unifiedGpsSeparation = distance2D(unifiedMapPose, gpsPose);
  const unifiedLidarSeparation = distance2D(unifiedMapPose, mapToSlamBase);
  const gridCanvas = useMemo(() => createOccupiedGridCanvas(grid), [grid]);
  const followZoom = DEFAULT_FOLLOW_ZOOM;

  const centerMapOnLatestFix = useCallback(() => {
    const map = mapRef.current;
    if (!map) {
      return;
    }

    const preferredPose = unifiedMapPose ?? gpsPose;
    const markerCenter = projectionAnchor && preferredPose ? localToMercator(projectionAnchor, preferredPose) : null;
    const center = markerCenter ?? (lonLat ? fromLonLat(lonLat) : null);
    if (!center) {
      return;
    }

    const view = map.getView();
    view.setCenter(center);
    if ((view.getZoom() ?? 0) < followZoom) {
      view.setZoom(followZoom);
    }
  }, [followZoom, gpsPose, lonLat, projectionAnchor, unifiedMapPose]);

  const restoreEditorDragPanInteractions = useCallback(() => {
    for (const { interaction, wasActive } of editorDisabledDragPanRef.current) {
      interaction.setActive(wasActive);
    }
    editorDisabledDragPanRef.current = [];
  }, []);

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
    if (
      projectionAnchor ||
      !selectedMap ||
      !Number.isFinite(selectedMap.datum_lat) ||
      !Number.isFinite(selectedMap.datum_lon) ||
      (selectedMap.datum_lat === 0 && selectedMap.datum_lon === 0)
    ) {
      return;
    }

    setProjectionAnchor({
      fixed: false,
      lat: selectedMap.datum_lat,
      localX: 0,
      localY: 0,
      lon: selectedMap.datum_lon,
      updatedAt: now,
    });
  }, [now, projectionAnchor, selectedMap]);

  useEffect(() => {
    if (!mapElementRef.current) {
      return undefined;
    }

    const mowerSource = new VectorSource();
    const positionSource = new VectorSource();
    const calibrationSource = new VectorSource();
    const actualTrackSource = new VectorSource();
    const editorSource = new VectorSource();
    const mbfPathSource = new VectorSource();
    const recordingOverlaySource = new VectorSource();
    const routeSource = new VectorSource();
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
    const editorLayer = new VectorLayer({
      source: editorSource,
      style: editorFeatureStyle,
      visible: false,
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
    const actualTrackLayer = new VectorLayer({
      source: actualTrackSource,
      style: actualTrackFeatureStyle,
      visible: false,
    });
    const recordingOverlayLayer = new VectorLayer({
      source: recordingOverlaySource,
      style: recordingOverlayFeatureStyle,
      visible: true,
    });
    const mbfPathLayer = new VectorLayer({
      source: mbfPathSource,
      style: mbfPathFeatureStyle,
      visible: false,
    });
    const routeLayer = new VectorLayer({
      source: routeSource,
      style: routeFeatureStyle,
      visible: false,
    });

    const map = new OlMap({
      layers: [
        satelliteLayer,
        slamLayer,
        mowerLayer,
        calibrationLayer,
        recordingOverlayLayer,
        routeLayer,
        mbfPathLayer,
        actualTrackLayer,
        editorLayer,
        scanLayer,
        new VectorLayer({
          source: positionSource,
          style: positionFeatureStyle,
        }),
      ],
      target: mapElementRef.current,
      view: new View({
        center: fromLonLat([0, 0]),
        zoom: 2,
      }),
    });

    mapRef.current = map;
    editorSourceRef.current = editorSource;
    mbfPathSourceRef.current = mbfPathSource;
    mowerSourceRef.current = mowerSource;
    positionSourceRef.current = positionSource;
    calibrationSourceRef.current = calibrationSource;
    actualTrackSourceRef.current = actualTrackSource;
    recordingOverlaySourceRef.current = recordingOverlaySource;
    routeSourceRef.current = routeSource;
    scanSourceRef.current = scanSource;
    slamSourceRef.current = slamSource;
    satelliteLayerRef.current = satelliteLayer;
    slamLayerRef.current = slamLayer;
    calibrationLayerRef.current = calibrationLayer;
    actualTrackLayerRef.current = actualTrackLayer;
    editorLayerRef.current = editorLayer;
    mbfPathLayerRef.current = mbfPathLayer;
    mowerLayerRef.current = mowerLayer;
    recordingOverlayLayerRef.current = recordingOverlayLayer;
    routeLayerRef.current = routeLayer;
    scanLayerRef.current = scanLayer;
    hasCenteredOnFirstFixRef.current = false;
    window.requestAnimationFrame(() => map.updateSize());

    return () => {
      map.setTarget(undefined);
      mapRef.current = null;
      editorSourceRef.current = null;
      mbfPathSourceRef.current = null;
      mowerSourceRef.current = null;
      positionSourceRef.current = null;
      calibrationSourceRef.current = null;
      actualTrackSourceRef.current = null;
      recordingOverlaySourceRef.current = null;
      routeSourceRef.current = null;
      scanSourceRef.current = null;
      slamSourceRef.current = null;
      satelliteLayerRef.current = null;
      slamLayerRef.current = null;
      calibrationLayerRef.current = null;
      actualTrackLayerRef.current = null;
      editorLayerRef.current = null;
      mbfPathLayerRef.current = null;
      mowerLayerRef.current = null;
      recordingOverlayLayerRef.current = null;
      routeLayerRef.current = null;
      scanLayerRef.current = null;
    };
  }, [
    config.satelliteArcGisFormat,
    config.satelliteArcGisLayers,
    config.satelliteArcGisRestUrl,
    config.satelliteAttribution,
    config.satelliteSourceType,
    config.satelliteTileUrl,
  ]);

  useEffect(() => {
    satelliteLayerRef.current?.setVisible(satelliteLayerVisible);
  }, [satelliteLayerVisible]);

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
    slamLayerRef.current?.setVisible(Boolean(!routeMode && layers.slamObstacles && projectionAnchor && grid && gridCanvas && slamToMap));
    slamLayerRef.current?.setOpacity(mapOpacity);
    slamSourceRef.current?.changed();
  }, [grid, gridCanvas, layers.slamObstacles, mapOpacity, projectionAnchor, routeMode, slamToMap]);

  useEffect(() => {
    const source = mowerSourceRef.current;
    const layer = mowerLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    layer.setVisible(mowerMapLayerVisible);
    if (!mowerMapLayerVisible || !projectionAnchor || !mapData) {
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
          showLabel: !routeMode && layers.labels,
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
          showLabel: !routeMode && layers.labels,
        }),
      );
    }

    source.addFeatures(features);
  }, [layers.labels, mapData, mowerMapLayerVisible, projectionAnchor, routeMode]);

  useEffect(() => {
    const source = recordingOverlaySourceRef.current;
    const layer = recordingOverlayLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    const overlayVisible = !routeMode && areaRecordingActive;
    layer.setVisible(overlayVisible);
    if (!overlayVisible || !projectionAnchor || !recordingOverlay?.polygons?.length) {
      return;
    }

    const features: Feature[] = [];
    for (const polygon of recordingOverlay.polygons) {
      const localPoints = (polygon.polygon?.points ?? [])
        .map((point) => ({ x: Number(point.x), y: Number(point.y) }))
        .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
      const coordinates = localPoints
        .map((point) => localToMercator(projectionAnchor, point))
        .filter((point): point is [number, number] => point !== null);
      if (coordinates.length < 2) {
        continue;
      }

      const properties = {
        closed: polygon.closed,
        lineWidth: polygon.line_width,
        overlayColor: polygon.color,
      };
      if (polygon.closed && coordinates.length >= 3) {
        features.push(
          new Feature({
            ...properties,
            geometry: new OlPolygon([closeRing(coordinates)]),
          }),
        );
      } else {
        features.push(
          new Feature({
            ...properties,
            geometry: new LineString(coordinates),
          }),
        );
      }
    }

    source.addFeatures(features);
  }, [areaRecordingActive, projectionAnchor, recordingOverlay, routeMode]);

  useEffect(() => {
    const source = routeSourceRef.current;
    const layer = routeLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    const routeViewerVisible = Boolean(routeMode && projectionAnchor && activeRoutePlan);
    const mapCoverageVisible = Boolean(!routeMode && layers.planProgress && projectionAnchor && liveRoutePlan);
    layer.setVisible(routeViewerVisible || mapCoverageVisible);
    if (!projectionAnchor) {
      return;
    }

    const features: Feature[] = [];
    if (routeMode) {
      if (!activeRoutePlan) {
        return;
      }

      if (routeLayers.basePath) {
        for (const path of activeRoutePlan.paths) {
          const coordinates = routePathBasePoints(path)
            .map((point) => localToMercator(projectionAnchor, point))
            .filter((point): point is [number, number] => point !== null);
          if (coordinates.length > 1) {
            features.push(new Feature({ geometry: new LineString(coordinates), kind: "routeBase" }));
          }
        }
      }

      if (routeLayers.toolPath) {
        for (const path of activeRoutePlan.paths) {
          const coordinates = routePathToolPoints(path, toolCenterOffsetM)
            .map((point) => localToMercator(projectionAnchor, point))
            .filter((point): point is [number, number] => point !== null);
          if (coordinates.length > 1) {
            features.push(new Feature({ geometry: new LineString(coordinates), kind: "routeTool" }));
          }
        }
      }

      if (routeLayers.outlineFootprints && selectedRoutePath?.is_outline) {
        for (const pose of selectedRoutePath.poses) {
          const ring = footprintToMercatorRing(projectionAnchor, routePoseTransform(pose), config.mowerFootprint);
          if (ring) {
            features.push(new Feature({ geometry: new OlPolygon([closeRing(ring)]), kind: "routeOutlineFootprint" }));
          }
        }
      }

      if (routeLayers.selectedFootprint && selectedRoutePose) {
        const ring = footprintToMercatorRing(projectionAnchor, routePoseTransform(selectedRoutePose), config.mowerFootprint);
        if (ring) {
          features.push(new Feature({ geometry: new OlPolygon([closeRing(ring)]), kind: "routeSelectedFootprint" }));
        }
        const toolPoint = routePoseToolPoint(selectedRoutePose, toolCenterOffsetM);
        const toolCoordinate = localToMercator(projectionAnchor, toolPoint);
        if (toolCoordinate) {
          features.push(new Feature({ geometry: new Point(toolCoordinate), kind: "routeToolPoint" }));
        }
      }
    } else if (layers.planProgress && liveRoutePlan) {
      for (const path of liveRoutePlan.paths) {
        const baseCoordinates = routePathBasePoints(path)
          .map((point) => localToMercator(projectionAnchor, point))
          .filter((point): point is [number, number] => point !== null);
        if (baseCoordinates.length > 1) {
          features.push(new Feature({ geometry: new LineString(baseCoordinates), kind: "planRouteBase" }));
        }

        const remainingCoordinates = routePathRemainingToolPoints(liveRoutePlan, path, toolCenterOffsetM)
          .map((point) => localToMercator(projectionAnchor, point))
          .filter((point): point is [number, number] => point !== null);
        if (remainingCoordinates.length > 1) {
          features.push(new Feature({ geometry: new LineString(remainingCoordinates), kind: "planRouteRemaining" }));
        }
      }
    }

    source.addFeatures(features);
  }, [
    activeRoutePlan,
    config.mowerFootprint,
    layers.planProgress,
    liveRoutePlan,
    projectionAnchor,
    routeLayers.basePath,
    routeLayers.outlineFootprints,
    routeLayers.selectedFootprint,
    routeLayers.toolPath,
    routeMode,
    selectedRoutePath,
    selectedRoutePose,
    toolCenterOffsetM,
  ]);

  useEffect(() => {
    const source = mbfPathSourceRef.current;
    const layer = mbfPathLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    const visible = Boolean(!routeMode && layers.mbfPath && projectionAnchor);
    layer.setVisible(visible);
    if (!visible || !projectionAnchor) {
      return;
    }

    const features: Feature[] = [];
    const globalCoordinates = navPathPoints(mbfGlobalPlan)
      .map((point) => localToMercator(projectionAnchor, point))
      .filter((point): point is [number, number] => point !== null);
    if (globalCoordinates.length > 1) {
      features.push(new Feature({ geometry: new LineString(globalCoordinates), kind: "mbfGlobalPlan" }));
    }

    const controllerCoordinates = navPathPoints(mbfControllerPlan)
      .map((point) => localToMercator(projectionAnchor, point))
      .filter((point): point is [number, number] => point !== null);
    if (controllerCoordinates.length > 1) {
      features.push(new Feature({ geometry: new LineString(controllerCoordinates), kind: "mbfControllerPlan" }));
    }

    source.addFeatures(features);
  }, [layers.mbfPath, mbfControllerPlan, mbfGlobalPlan, projectionAnchor, routeMode]);

  useEffect(() => {
    const source = actualTrackSourceRef.current;
    const layer = actualTrackLayerRef.current;
    if (!source || !layer) {
      return;
    }

    if (mowingOverlayActive && !actualTrackWasMowingRef.current) {
      actualTrackPointsRef.current = [];
    }
    actualTrackWasMowingRef.current = mowingOverlayActive;

    if (mowingOverlayActive && actualTrackPose) {
      const points = actualTrackPointsRef.current;
      const previous = points[points.length - 1] ?? null;
      if (!previous || distance2D(previous, actualTrackPose) >= ACTUAL_TRACK_MIN_STEP_M) {
        points.push({ x: actualTrackPose.x, y: actualTrackPose.y });
        if (points.length > ACTUAL_TRACK_MAX_POINTS) {
          points.splice(0, points.length - ACTUAL_TRACK_MAX_POINTS);
        }
      }
    }

    source.clear(true);
    layer.setVisible(actualTrackLayerVisible && Boolean(projectionAnchor) && actualTrackPointsRef.current.length > 0);
    if (!actualTrackLayerVisible || !projectionAnchor || actualTrackPointsRef.current.length === 0) {
      return;
    }

    const coordinates = actualTrackPointsRef.current
      .map((point) => localToMercator(projectionAnchor, point))
      .filter((point): point is [number, number] => point !== null);
    if (coordinates.length > 1) {
      source.addFeature(new Feature({ geometry: new LineString(coordinates), kind: "actualTrack" }));
    }
    if (coordinates.length > 0) {
      source.addFeature(new Feature({ geometry: new Point(coordinates[coordinates.length - 1]), kind: "actualTrackHead" }));
    }
  }, [actualTrackLayerVisible, actualTrackPose, mowingOverlayActive, projectionAnchor]);

  useEffect(() => {
    const source = editorSourceRef.current;
    const layer = editorLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    layer.setVisible(!routeMode && editorMode !== "view" && Boolean(projectionAnchor));
    if (routeMode || editorMode === "view" || !projectionAnchor) {
      return;
    }

    const features: Feature[] = [];
    if (editorMode === "paint" && pendingStroke.length > 0) {
      const coordinates = pendingStroke
        .map((point) => localToMercator(projectionAnchor, point))
        .filter((point): point is [number, number] => point !== null);
      if (coordinates.length === 1) {
        features.push(
          new Feature({
            brushDiameterM: brushDiameter,
            geometry: new Point(coordinates[0]),
            kind: "editorVertex",
            operation: paintTool,
          }),
        );
      } else if (coordinates.length > 1) {
        features.push(
          new Feature({
            brushDiameterM: brushDiameter,
            geometry: new LineString(coordinates),
            kind: "editorStroke",
            operation: paintTool,
          }),
        );
      }
    }

    if (editorMode === "polygon" && draftPolygon && draftPolygon.length >= 3) {
      const coordinates = draftPolygon
        .map((point) => localToMercator(projectionAnchor, point))
        .filter((point): point is [number, number] => point !== null);
      if (coordinates.length >= 3) {
        features.push(
          new Feature({
            geometry: new OlPolygon([closeRing(coordinates)]),
            kind: "editorPolygon",
          }),
        );
        features.push(
          new Feature({
            geometry: new MultiPoint(coordinates),
            kind: "editorVertex",
          }),
        );
      }
    }

    source.addFeatures(features);
  }, [brushDiameter, draftPolygon, editorMode, paintTool, pendingStroke, projectionAnchor, routeMode]);

  useEffect(() => {
    restoreEditorDragPanInteractions();
    const map = mapRef.current;
    if (!map || routeMode || editorMode === "view") {
      return undefined;
    }

    const disabledDragPanInteractions: Array<{ interaction: DragPan; wasActive: boolean }> = [];
    map.getInteractions().forEach((interaction) => {
      if (interaction instanceof DragPan) {
        disabledDragPanInteractions.push({
          interaction,
          wasActive: interaction.getActive(),
        });
        interaction.setActive(false);
      }
    });
    editorDisabledDragPanRef.current = disabledDragPanInteractions;

    return restoreEditorDragPanInteractions;
  }, [editorMode, restoreEditorDragPanInteractions, routeMode]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map || routeMode || !projectionAnchor || editorMode === "view") {
      return undefined;
    }

    const viewport = map.getViewport();
    const localFromEvent = (event: PointerEvent): MapPoint | null => {
      const coordinate = map.getCoordinateFromPixel(map.getEventPixel(event)) as [number, number];
      return mercatorToLocal(projectionAnchor, coordinate);
    };

    const stopMapEvent = (event: PointerEvent): void => {
      event.preventDefault();
      event.stopImmediatePropagation();
      event.stopPropagation();
    };

    const captureEditorPointer = (event: PointerEvent): void => {
      editorActivePointerIdRef.current = event.pointerId;
      if (!viewport.hasPointerCapture(event.pointerId)) {
        viewport.setPointerCapture(event.pointerId);
      }
    };

    const releaseEditorPointer = (event: PointerEvent): void => {
      if (viewport.hasPointerCapture(event.pointerId)) {
        viewport.releasePointerCapture(event.pointerId);
      }
      editorActivePointerIdRef.current = null;
    };

    const handlePointerDown = (event: PointerEvent): void => {
      if (!event.isPrimary || event.button !== 0) {
        stopMapEvent(event);
        return;
      }
      stopMapEvent(event);
      const local = localFromEvent(event);
      if (!local) {
        return;
      }
      if (editorMode === "paint") {
        editorDrawingRef.current = true;
        captureEditorPointer(event);
        setPendingStroke([local]);
        return;
      }

      const currentDraft = draftPolygonRef.current;
      if (!currentDraft || currentDraft.length < 3) {
        return;
      }
      if (polygonTool === "move") {
        const vertexIndex = nearestVertexIndex(currentDraft, local);
        if (vertexIndex >= 0) {
          editorDragVertexRef.current = vertexIndex;
          captureEditorPointer(event);
        }
      } else if (polygonTool === "add") {
        const insertIndex = nearestSegmentInsertIndex(currentDraft, local);
        if (insertIndex >= 0) {
          setDraftPolygon((current) => {
            if (!current) {
              return current;
            }
            const next = [...current];
            next.splice(insertIndex, 0, local);
            return next;
          });
        }
      } else if (polygonTool === "delete") {
        const vertexIndex = nearestVertexIndex(currentDraft, local);
        if (vertexIndex >= 0 && currentDraft.length > 3) {
          setDraftPolygon((current) => current?.filter((_, index) => index !== vertexIndex) ?? current);
        }
      }
    };

    const handlePointerMove = (event: PointerEvent): void => {
      const activePointerId = editorActivePointerIdRef.current;
      if (activePointerId !== null && activePointerId !== event.pointerId) {
        stopMapEvent(event);
        return;
      }
      const local = localFromEvent(event);
      if (!local) {
        return;
      }
      if (editorMode === "paint" && editorDrawingRef.current) {
        setPendingStroke((current) => {
          const last = current[current.length - 1];
          if (last && Math.hypot(last.x - local.x, last.y - local.y) < 0.04) {
            return current;
          }
          return [...current, local];
        });
        stopMapEvent(event);
        return;
      }

      const vertexIndex = editorDragVertexRef.current;
      if (editorMode === "polygon" && vertexIndex !== null) {
        setDraftPolygon((current) => {
          if (!current || vertexIndex < 0 || vertexIndex >= current.length) {
            return current;
          }
          const next = [...current];
          next[vertexIndex] = local;
          return next;
        });
        stopMapEvent(event);
      }
    };

    const handlePointerUp = (event: PointerEvent): void => {
      if (editorActivePointerIdRef.current !== null && editorActivePointerIdRef.current !== event.pointerId) {
        stopMapEvent(event);
        return;
      }
      if (editorDrawingRef.current || editorDragVertexRef.current !== null) {
        stopMapEvent(event);
      }
      releaseEditorPointer(event);
      editorDrawingRef.current = false;
      editorDragVertexRef.current = null;
    };

    const editorPointerOptions: AddEventListenerOptions = { capture: true };
    viewport.addEventListener("pointerdown", handlePointerDown, editorPointerOptions);
    viewport.addEventListener("pointermove", handlePointerMove, editorPointerOptions);
    viewport.addEventListener("pointerup", handlePointerUp, editorPointerOptions);
    viewport.addEventListener("pointercancel", handlePointerUp, editorPointerOptions);
    return () => {
      viewport.removeEventListener("pointerdown", handlePointerDown, editorPointerOptions);
      viewport.removeEventListener("pointermove", handlePointerMove, editorPointerOptions);
      viewport.removeEventListener("pointerup", handlePointerUp, editorPointerOptions);
      viewport.removeEventListener("pointercancel", handlePointerUp, editorPointerOptions);
      editorDrawingRef.current = false;
      editorDragVertexRef.current = null;
      editorActivePointerIdRef.current = null;
    };
  }, [editorMode, polygonTool, projectionAnchor, routeMode]);

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
            confidence: gpsConfidence ?? 0,
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
            confidence: lidarGlobalConfidence ?? lidarLocalConfidence ?? 0,
            geometry: new Point(coordinate),
            kind: "lidarRobot",
            showLabel: layers.labels,
            yaw: mapToSlamBase.yaw,
          }),
        );
      }
    }
    if (projectionAnchor && unifiedMapPose) {
      const footprintRing = footprintToMercatorRing(projectionAnchor, unifiedMapPose, config.mowerFootprint);
      const frontEdge = footprintFrontEdgeToMercator(projectionAnchor, unifiedMapPose, config.mowerFootprint);
      if (footprintRing) {
        features.push(
          new Feature({
            confidence: unifiedConfidence ?? 0,
            geometry: new OlPolygon([closeRing(footprintRing)]),
            kind: "finalFootprint",
            showLabel: layers.labels,
          }),
        );
      }
      if (frontEdge) {
        features.push(new Feature({ geometry: new LineString(frontEdge), kind: "finalFootprintFront" }));
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
    if (!hasCenteredOnFirstFixRef.current && (lonLat || gpsPose || unifiedMapPose)) {
      centerMapOnLatestFix();
      hasCenteredOnFirstFixRef.current = true;
    }
  }, [
    accuracy,
    centerMapOnLatestFix,
    gpsConfidence,
    gpsPose,
    layers.gpsAccuracy,
    layers.labels,
    lidarGlobalConfidence,
    lidarLocalConfidence,
    lonLat,
    mapToSlamBase,
    projectionAnchor,
    separation,
    config.mowerFootprint,
    unifiedConfidence,
    unifiedMapPose,
  ]);

  useEffect(() => {
    const source = calibrationSourceRef.current;
    const layer = calibrationLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    layer.setVisible(!routeMode && layers.calibration);
    if (routeMode || !layers.calibration || !projectionAnchor || !alignmentStatus?.boundary_pairs?.length) {
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
  }, [alignmentStatus?.boundary_pairs, layers.calibration, projectionAnchor, routeMode]);

  useEffect(() => {
    const source = scanSourceRef.current;
    const layer = scanLayerRef.current;
    if (!source || !layer) {
      return;
    }

    source.clear(true);
    layer.setVisible(!routeMode && layers.liveScan);
    if (routeMode || !layers.liveScan || !projectionAnchor || !scan || !scanToMap) {
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
  }, [layers.liveScan, projectionAnchor, routeMode, scan, scanClampMeters, scanToMap]);

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

  async function runMapCommand(command: () => Promise<{ message: string; success: boolean }>): Promise<void> {
    if (!ros || !connected || mapCommandPending || !mapMutationAllowed) {
      return;
    }

    setMapCommandPending(true);
    setMapCommandMessage(null);
    try {
      const response = await command();
      setMapCommandMessage(response.message || "Map command completed");
      if (!response.success) {
        throw new Error(response.message || "Map command failed");
      }
    } catch (serviceError) {
      setMapCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setMapCommandPending(false);
    }
  }

  function openCreateMapDialog(): void {
    setMapSelectorOpen(true);
    setMapNameDialog("create");
    window.requestAnimationFrame(() => newMapNameInputRef.current?.focus());
  }

  function openRenameMapDialog(): void {
    setMapSelectorOpen(true);
    setMapNameDialog("rename");
    window.requestAnimationFrame(() => renameMapNameInputRef.current?.focus());
  }

  function createMap(event: FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    const name = newMapName.trim();
    if (!name) {
      setMapCommandMessage("Enter a map name first");
      return;
    }
    void runMapCommand(async () => {
      const response = await callCreateMapService(ros as Ros, config.mapCreateService, name);
      if (response.success) {
        setNewMapName("");
      }
      return response;
    });
  }

  function selectMap(mapId: string): void {
    if (!mapId || mapId === selectedMap?.id) {
      return;
    }
    void runMapCommand(() => callSelectMapService(ros as Ros, config.mapSelectService, mapId));
  }

  function renameSelectedMap(event: FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    if (!selectedMap) {
      return;
    }
    const name = renameMapName.trim();
    if (!name) {
      setMapCommandMessage("Enter a map name first");
      return;
    }
    void runMapCommand(() => callRenameMapService(ros as Ros, config.mapRenameService, selectedMap.id, name));
  }

  function deleteSelectedMap(): void {
    if (!selectedMap) {
      return;
    }
    if (!window.confirm(`Delete map "${selectedMap.name}"?`)) {
      return;
    }
    void runMapCommand(() => callDeleteMapService(ros as Ros, config.mapDeleteService, selectedMap.id));
  }

  function makeMapEdit(operation: number): MapEditStroke {
    return {
      brush_diameter_m: brushDiameter,
      expected_map_hash: areaRecordingActive ? "" : selectedMap?.map_hash ?? "",
      expected_map_id: areaRecordingActive ? "" : selectedMap?.id ?? "",
      operation,
      path: rosPolygonFromPoints(pendingStroke),
      replacement_polygon: rosPolygonFromPoints(draftPolygon ?? []),
      target_area_id: areaRecordingActive ? "" : selectedEditArea?.id ?? "",
    };
  }

  function cancelCurrentEdit(): void {
    setPendingStroke([]);
    setDraftPolygon(selectedEditArea ? openMapRing(selectedEditArea.outline) : null);
    setEditCommandMessage(null);
    editorDrawingRef.current = false;
    editorDragVertexRef.current = null;
  }

  async function applyCurrentEdit(): Promise<void> {
    if (!ros || !connected || editCommandPending || !editMutationAllowed) {
      return;
    }
    const operation =
      editorMode === "polygon"
        ? EDIT_OPERATION_REPLACE_POLYGON
        : paintTool === "eraser"
          ? EDIT_OPERATION_BRUSH_ERASE
          : EDIT_OPERATION_BRUSH_ADD;
    if (editorMode === "paint" && pendingStroke.length === 0) {
      setEditCommandMessage("Draw a stroke first");
      return;
    }
    if (editorMode === "polygon" && (!draftPolygon || draftPolygon.length < 3)) {
      setEditCommandMessage("Polygon needs at least three points");
      return;
    }
    if (!areaRecordingActive && !selectedEditArea) {
      setEditCommandMessage("Select an area to edit");
      return;
    }

    setEditCommandPending(true);
    setEditCommandMessage(null);
    try {
      const edit = makeMapEdit(operation);
      const response = areaRecordingActive
        ? await callApplyRecordingEditService(ros, config.recordingEditService, edit)
        : await callApplyMapEditService(ros, config.mapEditService, edit);
      setEditCommandMessage(response.message || "Edit applied");
      if (!response.success) {
        throw new Error(response.message || "Edit failed");
      }
      setPendingStroke([]);
    } catch (serviceError) {
      setEditCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setEditCommandPending(false);
    }
  }

  function callAreaAction(action: string): void {
    if (!connected || !hasEnabledAction(action)) {
      return;
    }
    publishMowerAction(action);
  }

  function areaActionDisabled(action: string): boolean {
    return !connected || !hasEnabledAction(action);
  }

  function setLayer(layer: keyof LayerSettings, enabled: boolean): void {
    setLayers((current) => ({ ...current, [layer]: enabled }));
  }

  function setRouteLayer(layer: keyof RouteLayerSettings, enabled: boolean): void {
    setRouteLayers((current) => ({ ...current, [layer]: enabled }));
  }

  function stepRoutePose(delta: number): void {
    if (!selectedRoutePath) {
      return;
    }
    setSelectedRoutePoseIndex((current) => Math.max(0, Math.min(selectedRoutePath.poses.length - 1, current + delta)));
  }

  async function previewGeneratedRoute(): Promise<void> {
    if (!ros || routePreviewPending) {
      return;
    }
    setRoutePreviewPending(true);
    setRoutePreviewMessage("Generating preview...");
    setRouteSourceMode("live");
    try {
      const response = await callTriggerService(ros, config.routePlanPreviewService);
      setRoutePreviewMessage(response.message || (response.success ? "Route preview generated" : "Route preview failed"));
    } catch (previewError) {
      setRoutePreviewMessage(previewError instanceof Error ? previewError.message : String(previewError));
    } finally {
      setRoutePreviewPending(false);
    }
  }

  async function importRouteFile(file: File | null): Promise<void> {
    if (!file) {
      return;
    }
    try {
      const text = await file.text();
      const routePlan = parseImportedRoutePlanPayload(text);
      if (!routePlan || routePlan.paths.length === 0) {
        throw new Error("Unsupported route JSON");
      }
      setImportedRoutePlan(routePlan);
      setRouteSourceMode("import");
      setRouteImportMessage(`${file.name}: ${routePlan.paths.length} paths`);
    } catch (importError) {
      setRouteImportMessage(importError instanceof Error ? importError.message : String(importError));
    }
  }

  function clearImportedRoute(): void {
    setImportedRoutePlan(null);
    setRouteSourceMode("live");
    setRouteImportMessage(null);
  }

  function fitRouteToMap(): void {
    if (!mapRef.current || !projectionAnchor || !activeRoutePlan) {
      return;
    }
    const coordinates = activeRoutePlan.paths
      .flatMap((path) => routePathBasePoints(path))
      .map((point) => localToMercator(projectionAnchor, point))
      .filter((point): point is [number, number] => point !== null);
    if (coordinates.length === 0) {
      return;
    }
    mapRef.current.getView().fit(boundingExtent(coordinates), {
      duration: 250,
      maxZoom: 22,
      padding: [90, 420, 90, 90],
    });
  }

  const metricValues = useMemo(
    () => ({
      accuracy: Number.isFinite(accuracy) ? `${formatNumber(accuracy, accuracy < 1 ? 2 : 1)} m` : "--",
      actionsAge: formatAge(actionsStats.lastMessageAt, now),
      alignmentAge: formatAge(alignmentStats.lastMessageAt, now),
      anchor: projectionAnchor ? (projectionAnchor.fixed ? "RTK anchor" : "Approx anchor") : "No anchor",
      boundaryPath: formatMeters(alignmentStatus?.boundary_path_length_m, 1),
      boundarySamples:
        alignmentStatus?.boundary_sample_received_count && alignmentStatus.boundary_sample_received_count > (alignmentStatus?.boundary_sample_count ?? 0)
          ? `${alignmentStatus?.boundary_sample_count ?? 0}/${alignmentStatus.boundary_sample_received_count}`
          : String(alignmentStatus?.boundary_sample_count ?? 0),
      boundarySkipped: String(boundarySkipCount),
      confidenceAge: formatAge(confidenceStats.lastMessageAt, now),
      fusionAge: formatAge(fusionStats.lastMessageAt, now),
      fusionGpsWeight: formatPercent(fusionWeight(freshFusionStatus, "gps")),
      fusionLidarWeight: formatPercent(fusionWeight(freshFusionStatus, "lidar")),
      fusionPoseAge: formatAge(unifiedPoseStats.lastMessageAt, now),
      fusionReady:
        freshFusionStatus?.ready_for_navigation === true
          ? "Ready"
          : freshFusionStatus?.ready_for_navigation === false
            ? "Not ready"
            : "--",
      fusionState: statusLabel(freshFusionStatus?.state),
      gpsAge: formatAge(fixStats.lastMessageAt, now),
      gpsConfidence: formatPercent(gpsConfidence),
      gpsSigma: formatMeters(confidenceStatus?.gps.position_sigma_m, 2),
      gpsYawSigma:
        confidenceStatus?.gps.yaw_sigma_rad === null || confidenceStatus?.gps.yaw_sigma_rad === undefined
          ? "--"
          : `${formatNumber((confidenceStatus.gps.yaw_sigma_rad * 180) / Math.PI, 1)} deg`,
      lidarAge: formatAge(slamMapStats.lastMessageAt, now),
      lidarGlobalConfidence: formatPercent(lidarGlobalConfidence),
      lidarLocalConfidence: formatPercent(lidarLocalConfidence),
      lidarSigma: formatMeters(confidenceStatus?.lidar.position_sigma_local_m, 2),
      lidarYawSigma:
        confidenceStatus?.lidar.yaw_sigma_local_rad === null || confidenceStatus?.lidar.yaw_sigma_local_rad === undefined
          ? "--"
          : `${formatNumber((confidenceStatus.lidar.yaw_sigma_local_rad * 180) / Math.PI, 1)} deg`,
      unifiedConfidence: formatPercent(unifiedConfidence),
      unifiedGpsSeparation: Number.isFinite(unifiedGpsSeparation) ? `${formatNumber(unifiedGpsSeparation, 2)} m` : "--",
      unifiedLidarSeparation: Number.isFinite(unifiedLidarSeparation) ? `${formatNumber(unifiedLidarSeparation, 2)} m` : "--",
      unifiedSigma: formatMeters(freshFusionStatus?.position_sigma_m, 2),
      unifiedYawSigma:
        freshFusionStatus?.yaw_sigma_rad === null || freshFusionStatus?.yaw_sigma_rad === undefined
          ? "--"
          : `${formatNumber((freshFusionStatus.yaw_sigma_rad * 180) / Math.PI, 1)} deg`,
      maxResidual: formatMeters(alignmentStatus?.residual_max_m, 2),
      mapCatalogAge: formatAge(mapCatalogStats.lastMessageAt, now),
      outliers: String(alignmentStatus?.outlier_count ?? 0),
      p95Residual: formatMeters(alignmentStatus?.residual_p95_m, 2),
      residual: formatMeters(alignmentStatus?.residual_m, 2),
      robotStateAge: formatAge(robotStateStats.lastMessageAt, now),
      recordingOverlayAge: formatAge(recordingOverlayStats.lastMessageAt, now),
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
      actionsStats.lastMessageAt,
      alignmentStats.lastMessageAt,
      alignmentStatus?.alignment_source,
      alignmentStatus?.boundary_path_length_m,
      alignmentStatus?.boundary_sample_received_count,
      alignmentStatus?.boundary_sample_count,
      alignmentStatus?.outlier_count,
      alignmentStatus?.residual_m,
      alignmentStatus?.residual_max_m,
      alignmentStatus?.residual_p95_m,
      alignmentStatus?.scale_diagnostic,
      alignmentStatus?.yaw_offset_rad,
      boundarySkipCount,
      confidenceStats.lastMessageAt,
      confidenceStatus?.gps.position_sigma_m,
      confidenceStatus?.gps.yaw_sigma_rad,
      confidenceStatus?.lidar.position_sigma_local_m,
      confidenceStatus?.lidar.yaw_sigma_local_rad,
      fixStats.lastMessageAt,
      fusionStats.lastMessageAt,
      freshFusionStatus,
      freshFusionStatus?.position_sigma_m,
      freshFusionStatus?.ready_for_navigation,
      freshFusionStatus?.state,
      freshFusionStatus?.yaw_sigma_rad,
      gpsConfidence,
      lidarGlobalConfidence,
      lidarLocalConfidence,
      mapCatalogStats.lastMessageAt,
      now,
      projectionAnchor,
      recordingOverlayStats.lastMessageAt,
      robotStateStats.lastMessageAt,
      scanStats.hz,
      separation,
      slamMapStats.lastMessageAt,
      unifiedConfidence,
      unifiedGpsSeparation,
      unifiedLidarSeparation,
      unifiedPoseStats.lastMessageAt,
    ],
  );

  const autoCollectingActive = hasEnabledAction(ACTION_AUTO_COLLECT_DISABLE);
  const driveInputLabel = directManualInputActive ? "Direct Bluetooth" : "Web Gamepad";
  const gamepadLabel = directManualInputActive
    ? directManualInputConnected
      ? mowerInputStatus?.direct_blade_hold_active
        ? "Blade hold"
        : "Controller live"
      : "Waiting"
    : areaControl.gamepad.connected
      ? areaControl.gamepad.bladeComboPressed
        ? "Blade hold"
        : areaControl.gamepad.driving
          ? "Driving"
          : "Ready"
      : "No controller";
  const gamepadConnected = directManualInputActive ? directManualInputConnected : areaControl.gamepad.connected;

  return (
    <main className={`combined-map-workspace ${routeMode ? "is-route-mode" : ""}`}>
      {!routeMode && (
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

        <section className="panel-section map-editor-section">
          <div className="layer-heading">
            <Edit3 size={18} aria-hidden="true" />
            <span>Edit Map</span>
          </div>
          <div className="segmented-control">
            {(["view", "paint", "polygon"] as EditorMode[]).map((mode) => (
              <button
                className={editorMode === mode ? "is-active" : ""}
                disabled={areaRecordingActive && mode === "polygon"}
                key={mode}
                type="button"
                onClick={() => setEditorMode(mode)}
              >
                {mode === "view" ? <Hand size={16} aria-hidden="true" /> : mode === "paint" ? <Edit3 size={16} aria-hidden="true" /> : <Route size={16} aria-hidden="true" />}
                <span>{mode}</span>
              </button>
            ))}
          </div>
          {!editMutationAllowed && <div className="status-banner">Edits require IDLE or paused area recording</div>}
          {editorMode !== "view" && !areaRecordingActive && (
            <>
              <label htmlFor="edit-area-select">Target area</label>
              <select
                className="map-select"
                disabled={!savedMapEditAllowed || editCommandPending || editableAreas.length === 0}
                id="edit-area-select"
                value={selectedEditArea?.id ?? ""}
                onChange={(event) => setSelectedEditAreaId(event.target.value)}
              >
                {editableAreas.length === 0 && <option value="">No saved areas</option>}
                {editableAreas.map((area, index) => (
                  <option key={area.id} value={area.id}>
                    {area.properties?.name || `${mowerAreaType(area)} ${index + 1}`}
                  </option>
                ))}
              </select>
            </>
          )}
          {editorMode === "paint" && (
            <>
              <div className="tool-button-row">
                <button className={paintTool === "pen" ? "is-active" : ""} type="button" onClick={() => setPaintTool("pen")}>
                  <Edit3 size={16} aria-hidden="true" />
                  <span>Pen</span>
                </button>
                <button className={paintTool === "eraser" ? "is-active" : ""} type="button" onClick={() => setPaintTool("eraser")}>
                  <Trash2 size={16} aria-hidden="true" />
                  <span>Eraser</span>
                </button>
              </div>
              <div className="tool-button-row">
                {(["small", "mower", "large"] as BrushSizeKey[]).map((sizeKey) => (
                  <button
                    className={brushSizeKey === sizeKey ? "is-active" : ""}
                    key={sizeKey}
                    type="button"
                    onClick={() => setBrushSizeKey(sizeKey)}
                  >
                    <span>{BRUSH_SIZES[sizeKey].label}</span>
                  </button>
                ))}
              </div>
            </>
          )}
          {editorMode === "polygon" && (
            <div className="tool-button-row">
              <button className={polygonTool === "move" ? "is-active" : ""} type="button" onClick={() => setPolygonTool("move")}>
                <Hand size={16} aria-hidden="true" />
                <span>Move</span>
              </button>
              <button className={polygonTool === "add" ? "is-active" : ""} type="button" onClick={() => setPolygonTool("add")}>
                <Plus size={16} aria-hidden="true" />
                <span>Add</span>
              </button>
              <button className={polygonTool === "delete" ? "is-active" : ""} type="button" onClick={() => setPolygonTool("delete")}>
                <X size={16} aria-hidden="true" />
                <span>Remove</span>
              </button>
            </div>
          )}
          {editorMode !== "view" && (
            <div className="editor-command-row">
              <button
                aria-label="Undo local stroke"
                className="icon-button"
                disabled={editCommandPending || (pendingStroke.length === 0 && editorMode !== "polygon")}
                title="Undo local stroke"
                type="button"
                onClick={() => setPendingStroke([])}
              >
                <LogOut size={18} aria-hidden="true" />
              </button>
              <button
                aria-label="Cancel edit"
                className="icon-button"
                disabled={editCommandPending}
                title="Cancel edit"
                type="button"
                onClick={cancelCurrentEdit}
              >
                <X size={18} aria-hidden="true" />
              </button>
              <button
                className="command-button"
                disabled={!editMutationAllowed || editCommandPending || (editorMode === "paint" && pendingStroke.length === 0)}
                type="button"
                onClick={() => void applyCurrentEdit()}
              >
                <Save size={18} aria-hidden="true" />
                <span>{editCommandPending ? "Applying" : areaRecordingActive ? "Queue Edit" : "Apply Edit"}</span>
              </button>
            </div>
          )}
          {editCommandMessage && <div className="status-banner">{editCommandMessage}</div>}
        </section>

        <section className="panel-section area-recording-summary">
          <div className="layer-heading">
            <Route size={18} aria-hidden="true" />
            <span>Area Recording</span>
          </div>
          <div className={`sensor-pill is-${areaRecordingActive ? "functioning" : hasEnabledAction(ACTION_START_AREA_RECORDING) ? "waiting" : "offline"}`}>
            <CircleDot size={18} aria-hidden="true" />
            <span>{areaRecordingActive ? currentSubState || "AREA_RECORDING" : currentState}</span>
          </div>
          <label className="layer-toggle area-recording-pose-toggle" title="Area recording pose source">
            <input
              type="checkbox"
              checked={areaRecordingUseFusedPose}
              disabled={!connected || recordingInProgress || areaRecordingPoseSourcePending}
              onChange={(event) => void setAreaRecordingPoseSource(event.target.checked)}
            />
            <span>{areaRecordingPoseSourcePending ? "Switching" : recordingPoseSourceLabel}</span>
          </label>
          {!areaRecordingActive && (
            <div className="control-row">
              <button
                className="command-button"
                disabled={areaActionDisabled(ACTION_START_AREA_RECORDING)}
                type="button"
                onClick={() => callAreaAction(ACTION_START_AREA_RECORDING)}
              >
                <CircleDot size={18} aria-hidden="true" />
                <span>Area Recording</span>
              </button>
            </div>
          )}
          {areaRecordingActive && (
            <div className="area-recording-mini-status">
              <div>
                <span>Actions</span>
                <strong>{enabledActionIds.size}</strong>
              </div>
              <div>
                <span>Overlay</span>
                <strong>{recordingOverlay?.polygons?.length ?? 0}</strong>
              </div>
            </div>
          )}
          {areaCommandMessage && <div className="status-banner">{areaCommandMessage}</div>}
        </section>

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

        <section className="panel-section confidence-summary">
          <div className="layer-heading">
            <Activity size={18} aria-hidden="true" />
            <span>Confidence</span>
          </div>
          <div className="confidence-list">
            <div className={`confidence-row is-${confidenceClass(gpsConfidence)}`}>
              <div>
                <span>GPS position</span>
                <strong>{metricValues.gpsConfidence}</strong>
              </div>
              <div className="confidence-track">
                <span style={{ width: `${Math.max(0, Math.min(1, gpsConfidence ?? 0)) * 100}%` }} />
              </div>
            </div>
            <div className={`confidence-row is-${confidenceClass(lidarLocalConfidence)}`}>
              <div>
                <span>LIDAR local</span>
                <strong>{metricValues.lidarLocalConfidence}</strong>
              </div>
              <div className="confidence-track">
                <span style={{ width: `${Math.max(0, Math.min(1, lidarLocalConfidence ?? 0)) * 100}%` }} />
              </div>
            </div>
            <div className={`confidence-row is-${confidenceClass(lidarGlobalConfidence)}`}>
              <div>
                <span>LIDAR on map</span>
                <strong>{metricValues.lidarGlobalConfidence}</strong>
              </div>
              <div className="confidence-track">
                <span style={{ width: `${Math.max(0, Math.min(1, lidarGlobalConfidence ?? 0)) * 100}%` }} />
              </div>
            </div>
            <div className={`confidence-row is-${confidenceClass(unifiedConfidence)}`}>
              <div>
                <span>Unified final</span>
                <strong>{metricValues.unifiedConfidence}</strong>
              </div>
              <div className="confidence-track">
                <span style={{ width: `${Math.max(0, Math.min(1, unifiedConfidence ?? 0)) * 100}%` }} />
              </div>
            </div>
          </div>
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
            <input checked={layers.planProgress} type="checkbox" onChange={(event) => setLayer("planProgress", event.target.checked)} />
            <span>Coverage Route</span>
          </label>
          <label className="layer-toggle">
            <input checked={layers.mbfPath} type="checkbox" onChange={(event) => setLayer("mbfPath", event.target.checked)} />
            <span>MBF Path</span>
          </label>
          <label className="layer-toggle">
            <input checked={layers.actualTrack} type="checkbox" onChange={(event) => setLayer("actualTrack", event.target.checked)} />
            <span>Actual Track</span>
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
            <span>Unified</span>
            <strong>{metricValues.fusionReady}</strong>
          </div>
          <div className="metric">
            <span>Final-GPS</span>
            <strong>{metricValues.unifiedGpsSeparation}</strong>
          </div>
          <div className="metric">
            <span>Final-LIDAR</span>
            <strong>{metricValues.unifiedLidarSeparation}</strong>
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
              <span>Skipped</span>
              <strong>{metricValues.boundarySkipped}</strong>
            </div>
            <div>
              <span>Outliers</span>
              <strong>{metricValues.outliers}</strong>
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
          {alignmentStatus?.outlier_warning && <div className="warning-banner">Calibration outliers rejected</div>}
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
            <div className={`tf-chip is-${tfState(mapToUnifiedBaseTf)}`}>
              <span>{`map -> ${config.localizationFusionBaseFrame}`}</span>
              <strong>{tfState(mapToUnifiedBaseTf)}</strong>
            </div>
            <div>
              <span>Map age</span>
              <strong>{metricValues.lidarAge}</strong>
            </div>
            <div>
              <span>Robot state age</span>
              <strong>{metricValues.robotStateAge}</strong>
            </div>
            <div>
              <span>Actions age</span>
              <strong>{metricValues.actionsAge}</strong>
            </div>
            <div>
              <span>Overlay age</span>
              <strong>{metricValues.recordingOverlayAge}</strong>
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
              <span>Confidence age</span>
              <strong>{metricValues.confidenceAge}</strong>
            </div>
            <div>
              <span>Fusion status age</span>
              <strong>{metricValues.fusionAge}</strong>
            </div>
            <div>
              <span>Fusion pose age</span>
              <strong>{metricValues.fusionPoseAge}</strong>
            </div>
            <div>
              <span>Fusion state</span>
              <strong>{metricValues.fusionState}</strong>
            </div>
            <div>
              <span>Navigation ready</span>
              <strong>{metricValues.fusionReady}</strong>
            </div>
            <div>
              <span>Unified sigma</span>
              <strong>{metricValues.unifiedSigma}</strong>
            </div>
            <div>
              <span>Unified yaw sigma</span>
              <strong>{metricValues.unifiedYawSigma}</strong>
            </div>
            <div>
              <span>Fusion GPS weight</span>
              <strong>{metricValues.fusionGpsWeight}</strong>
            </div>
            <div>
              <span>Fusion LIDAR weight</span>
              <strong>{metricValues.fusionLidarWeight}</strong>
            </div>
            <div>
              <span>GPS confidence state</span>
              <strong>{confidenceStatus?.gps.state?.replace(/_/g, " ") || "--"}</strong>
            </div>
            <div>
              <span>GPS sigma</span>
              <strong>{metricValues.gpsSigma}</strong>
            </div>
            <div>
              <span>GPS yaw sigma</span>
              <strong>{metricValues.gpsYawSigma}</strong>
            </div>
            <div>
              <span>LIDAR confidence state</span>
              <strong>{confidenceStatus?.lidar.state?.replace(/_/g, " ") || "--"}</strong>
            </div>
            <div>
              <span>LIDAR sigma</span>
              <strong>{metricValues.lidarSigma}</strong>
            </div>
            <div>
              <span>LIDAR yaw sigma</span>
              <strong>{metricValues.lidarYawSigma}</strong>
            </div>
            <div>
              <span>Alignment confidence</span>
              <strong>{formatPercent(confidenceStatus?.alignment.confidence)}</strong>
            </div>
            <div>
              <span>Agreement</span>
              <strong>{formatPercent(confidenceStatus?.agreement.consistency_score)}</strong>
            </div>
            <div>
              <span>TF Hz</span>
              <strong>{formatNumber(tfStats.hz, 1)}</strong>
            </div>
            <div>
              <span>Mower map</span>
              <strong>{mowerMapStats.messageCount}</strong>
            </div>
            <div>
              <span>Map catalog age</span>
              <strong>{metricValues.mapCatalogAge}</strong>
            </div>
          </div>
          {confidenceStatus && (
            <div className="confidence-detail-stack">
              <div>
                <span>GPS components</span>
                <strong>{componentSummary(confidenceStatus.gps.components)}</strong>
              </div>
              <div>
                <span>GPS reasons</span>
                <strong>{reasonSummary(confidenceStatus.gps.reasons)}</strong>
              </div>
              <div>
                <span>LIDAR components</span>
                <strong>{componentSummary(confidenceStatus.lidar.components)}</strong>
              </div>
              <div>
                <span>LIDAR reasons</span>
                <strong>{reasonSummary(confidenceStatus.lidar.reasons)}</strong>
              </div>
            </div>
          )}
          {freshFusionStatus && (
            <div className="confidence-detail-stack">
              <div>
                <span>Fusion recommended action</span>
                <strong>{statusLabel(freshFusionStatus.recommended_action)}</strong>
              </div>
              <div>
                <span>Fusion reasons</span>
                <strong>{reasonSummary(freshFusionStatus.reasons ?? freshFusionStatus.rejections)}</strong>
              </div>
            </div>
          )}
        </details>

        {error && <div className="error-banner">{error}</div>}
      </aside>
      )}

      <section className={`combined-map-stage ${!routeMode ? "has-map-selector" : ""}`}>
        {!routeMode && (
          <div className="map-selector-bar" ref={mapSelectorRef}>
            <div className="map-selector-bar-inner">
              <div className="map-selector-closed">
                <button
                  aria-controls="map-selector-popover"
                  aria-expanded={mapSelectorOpen}
                  className="map-selector-pill"
                  type="button"
                  onClick={() => setMapSelectorOpen((current) => !current)}
                >
                  <MapIcon size={17} aria-hidden="true" />
                  <span className="map-selector-pill-text">{selectedMap?.name ?? "No map"}</span>
                  {selectedMapFar && (
                    <span className="map-selector-warning-chip" title={`${formatMeters(selectedMapDistance, 0)} from current pose`}>
                      <AlertTriangle size={14} aria-hidden="true" />
                    </span>
                  )}
                  <ChevronDown size={16} aria-hidden="true" />
                </button>
              </div>

              {mapSelectorOpen && (
                <div className="map-selector-popover" id="map-selector-popover" role="dialog" aria-label="Saved maps">
                  <select
                    aria-label="Saved maps"
                    className="map-select map-selector-select"
                    disabled={!mapMutationAllowed || mapCommandPending || !mapCatalog?.maps.length}
                    value={selectedMap?.id ?? ""}
                    onChange={(event) => selectMap(event.target.value)}
                  >
                    {!mapCatalog?.maps.length && <option value="">No maps</option>}
                    {mapCatalog?.maps.map((summary) => (
                      <option key={summary.id} value={summary.id}>
                        {summary.name}
                      </option>
                    ))}
                  </select>

                  <div className="map-selector-action-row">
                    <button
                      className="command-button"
                      disabled={!mapMutationAllowed || mapCommandPending}
                      type="button"
                      onClick={openCreateMapDialog}
                    >
                      <Plus size={18} aria-hidden="true" />
                      <span>Create</span>
                    </button>
                    <button
                      className="command-button is-secondary"
                      disabled={!mapMutationAllowed || mapCommandPending || !selectedMap}
                      type="button"
                      onClick={openRenameMapDialog}
                    >
                      <Edit3 size={18} aria-hidden="true" />
                      <span>Rename</span>
                    </button>
                    <button
                      className="command-button is-danger"
                      disabled={!mapMutationAllowed || mapCommandPending || !selectedMap}
                      type="button"
                      onClick={deleteSelectedMap}
                    >
                      <Trash2 size={18} aria-hidden="true" />
                      <span>Delete</span>
                    </button>
                  </div>

                  {selectedMapFar && (
                    <div className="warning-banner map-selector-inline-banner">
                      <AlertTriangle size={16} aria-hidden="true" />
                      <span>{formatMeters(selectedMapDistance, 0)} from current pose</span>
                    </div>
                  )}
                  {!mapMutationAllowed && <div className="status-banner map-selector-inline-banner">Map edits require IDLE state</div>}
                  {mapCommandMessage && <div className="status-banner map-selector-inline-banner">{mapCommandMessage}</div>}

                  <div className="map-selector-details">
                    <div>
                      <span>Mow</span>
                      <strong>{selectedMap?.mowing_area_count ?? 0}</strong>
                    </div>
                    <div>
                      <span>Dock</span>
                      <strong>{selectedMap?.has_docking_station ? "Yes" : "No"}</strong>
                    </div>
                    <div>
                      <span>Updated</span>
                      <strong>{formatMapTimestamp(selectedMap?.updated_at)}</strong>
                    </div>
                  </div>
                </div>
              )}
            </div>
          </div>
        )}
        <div className="combined-map-shell">
          <div className={`gps-map ${!routeMode && editorMode !== "view" ? "is-editor-active" : ""}`} ref={mapElementRef} />
          {!lonLat && !projectionAnchor && (
            <div className="gps-empty-state">
              <strong>No GPS projection anchor</strong>
              <span>{config.gpsFixTopic}</span>
            </div>
          )}
          <button
            aria-label="Center map on latest robot pose"
            className="gps-map-control-button"
            disabled={!lonLat && !gpsPose && !unifiedMapPose}
            onClick={centerMapOnLatestFix}
            title="Center map on latest robot pose"
            type="button"
          >
            <Crosshair size={20} aria-hidden="true" />
          </button>
          {!routeMode && <div className="map-marker-legend" aria-label="Robot marker legend">
            <span className="legend-marker is-gps">GPS</span>
            <span className="legend-marker is-lidar">LIDAR</span>
            <span className="legend-marker is-final">FOOTPRINT</span>
            <strong>{metricValues.separation}</strong>
          </div>}
          {routeMode && (
            <div className="route-map-panel" aria-label="Route viewer controls">
              <div className="route-panel-header">
                <div>
                  <span className="eyebrow">Route</span>
                  <strong>{routeStatus}</strong>
                </div>
                <button className="icon-button" type="button" title="Fit route" aria-label="Fit route" onClick={fitRouteToMap}>
                  <Crosshair size={18} aria-hidden="true" />
                </button>
              </div>

              <div className="segmented-control">
                <button
                  className={routeSourceMode === "live" ? "is-active" : ""}
                  type="button"
                  onClick={() => setRouteSourceMode("live")}
                >
                  <Route size={16} aria-hidden="true" />
                  <span>Live</span>
                </button>
                <button
                  className={routeSourceMode === "import" ? "is-active" : ""}
                  type="button"
                  onClick={() => setRouteSourceMode("import")}
                >
                  <Upload size={16} aria-hidden="true" />
                  <span>Import</span>
                </button>
              </div>

              <div className="route-file-row">
                <button
                  className="command-button"
                  disabled={!connected || !ros || routePreviewPending}
                  type="button"
                  onClick={() => void previewGeneratedRoute()}
                >
                  <Route size={18} aria-hidden="true" />
                  <span>{routePreviewPending ? "Previewing" : "Preview"}</span>
                </button>
                <label className="command-button route-file-button">
                  <Upload size={18} aria-hidden="true" />
                  <span>JSON</span>
                  <input
                    accept=".json,application/json"
                    type="file"
                    onChange={(event) => {
                      void importRouteFile(event.target.files?.[0] ?? null);
                      event.currentTarget.value = "";
                    }}
                  />
                </label>
                <button className="command-button is-secondary" disabled={!routeImportActive} type="button" onClick={clearImportedRoute}>
                  <X size={18} aria-hidden="true" />
                  <span>Clear</span>
                </button>
              </div>

              <div className="route-layer-grid">
                <label className="layer-toggle">
                  <input checked={routeLayers.satellite} type="checkbox" onChange={(event) => setRouteLayer("satellite", event.target.checked)} />
                  <span>Satellite</span>
                </label>
                <label className="layer-toggle">
                  <input checked={routeLayers.mowerMap} type="checkbox" onChange={(event) => setRouteLayer("mowerMap", event.target.checked)} />
                  <span>Mower Map</span>
                </label>
                <label className="layer-toggle">
                  <input checked={routeLayers.basePath} type="checkbox" onChange={(event) => setRouteLayer("basePath", event.target.checked)} />
                  <span>Base Link</span>
                </label>
                <label className="layer-toggle">
                  <input checked={routeLayers.toolPath} type="checkbox" onChange={(event) => setRouteLayer("toolPath", event.target.checked)} />
                  <span>Cutting Tool</span>
                </label>
                <label className="layer-toggle">
                  <input checked={routeLayers.selectedFootprint} type="checkbox" onChange={(event) => setRouteLayer("selectedFootprint", event.target.checked)} />
                  <span>Footprint Pose</span>
                </label>
                <label className="layer-toggle">
                  <input checked={routeLayers.outlineFootprints} type="checkbox" onChange={(event) => setRouteLayer("outlineFootprints", event.target.checked)} />
                  <span>Outline Footprints</span>
                </label>
                <label className="layer-toggle">
                  <input checked={routeLayers.actualTrack} type="checkbox" onChange={(event) => setRouteLayer("actualTrack", event.target.checked)} />
                  <span>Actual Track</span>
                </label>
              </div>

              <label className="route-field" htmlFor="route-path-select">
                <span>Path</span>
                <select
                  id="route-path-select"
                  disabled={!activeRoutePlan || activeRoutePlan.paths.length === 0}
                  value={selectedRoutePathListIndex}
                  onChange={(event) => {
                    const nextIndex = Number(event.target.value);
                    setSelectedRoutePathListIndex(nextIndex);
                    setSelectedRoutePoseIndex(0);
                  }}
                >
                  {(activeRoutePlan?.paths ?? []).map((path, index) => (
                    <option key={`${path.path_index}-${index}`} value={index}>
                      {path.path_index}: {path.is_outline ? "outline" : "fill"} {path.label} ({path.poses.length})
                    </option>
                  ))}
                </select>
              </label>

              <div className="route-pose-controls">
                <button className="icon-button" disabled={!selectedRoutePath || selectedRoutePoseIndex <= 0} type="button" aria-label="Previous pose" onClick={() => stepRoutePose(-1)}>
                  <ChevronLeft size={18} aria-hidden="true" />
                </button>
                <input
                  aria-label="Route pose"
                  disabled={!selectedRoutePath}
                  max={Math.max(0, selectedRoutePoseCount - 1)}
                  min={0}
                  type="range"
                  value={selectedRoutePoseIndex}
                  onChange={(event) => setSelectedRoutePoseIndex(Number(event.target.value))}
                />
                <button className="icon-button" disabled={!selectedRoutePath || selectedRoutePoseIndex >= selectedRoutePoseCount - 1} type="button" aria-label="Next pose" onClick={() => stepRoutePose(1)}>
                  <ChevronRight size={18} aria-hidden="true" />
                </button>
              </div>

              <div className="route-readout">
                <div>
                  <span>Path</span>
                  <strong>{selectedRoutePath ? `${selectedRoutePath.path_index}` : "--"}</strong>
                </div>
                <div>
                  <span>Pose</span>
                  <strong>{selectedRoutePose ? `${selectedRoutePose.pose_index}` : "--"}</strong>
                </div>
                <div>
                  <span>X/Y</span>
                  <strong>{selectedRoutePose ? `${formatNumber(selectedRoutePose.x, 2)}, ${formatNumber(selectedRoutePose.y, 2)}` : "--"}</strong>
                </div>
                <div>
                  <span>Yaw</span>
                  <strong>{selectedRoutePose ? `${formatNumber((selectedRoutePose.yaw * 180) / Math.PI, 1)} deg` : "--"}</strong>
                </div>
                <div>
                  <span>Tool</span>
                  <strong>
                    {selectedRoutePose
                      ? `${formatNumber(routePoseToolPoint(selectedRoutePose, toolCenterOffsetM).x, 2)}, ${formatNumber(routePoseToolPoint(selectedRoutePose, toolCenterOffsetM).y, 2)}`
                      : "--"}
                  </strong>
                </div>
                <div>
                  <span>Tool yaw</span>
                  <strong>{selectedRoutePose ? `${formatNumber((routePoseToolYaw(selectedRoutePose) * 180) / Math.PI, 1)} deg` : "--"}</strong>
                </div>
              </div>

              {routeLayers.outlineFootprints && !selectedRouteIsOutline && (
                <div className="status-banner">Select an outline path to view all outline footprints</div>
              )}
              {routePreviewMessage && <div className="status-banner">{routePreviewMessage}</div>}
              {routeImportMessage && <div className="status-banner">{routeImportMessage}</div>}
              <div className="route-mini-status">
                <span>{activeRoutePlan?.paths.length ?? 0} paths</span>
                <span>{formatAge(routePlanStats.lastMessageAt, now)}</span>
              </div>
            </div>
          )}
          {!routeMode && areaRecordingActive && (
            <div className="area-recording-map-panel" aria-label="Area recording controls">
              <div className="area-recording-drive">
                <div className="area-recording-panel-header">
                  <div>
                    <span className="eyebrow">Drive</span>
                    <strong>{driveInputLabel}</strong>
                  </div>
                  <div className={`gamepad-chip ${gamepadConnected ? "is-connected" : "is-offline"}`}>
                    <Gamepad2 size={17} aria-hidden="true" />
                    <span>{gamepadLabel}</span>
                  </div>
                </div>
                <AreaJoystickPad
                  command={areaControl.joystickCommand}
                  disabled={!connected || !areaRecordingActive || !webManualInputActive}
                  onCommand={areaControl.updateTouchCommand}
                  onRelease={areaControl.clearTouchCommand}
                />
                <button
                  className={`blade-hold-button ${areaControl.bladeHoldActive ? "is-active" : ""}`}
                  disabled={!connected || !areaRecordingActive || !webManualInputActive}
                  onPointerCancel={() => areaControl.setTouchBladePressed(false)}
                  onPointerDown={() => areaControl.setTouchBladePressed(true)}
                  onPointerLeave={() => areaControl.setTouchBladePressed(false)}
                  onPointerUp={() => areaControl.setTouchBladePressed(false)}
                  type="button"
                >
                  <Hand size={18} aria-hidden="true" />
                  <span>{areaControl.bladeHoldActive ? "Blade Live" : "Blade Hold"}</span>
                </button>
              </div>

              <div className="area-recording-actions">
                <div className="area-recording-panel-header">
                  <div>
                    <span className="eyebrow">Recording</span>
                    <strong>{`${recordingInProgress ? "Recording" : "Ready"} · ${recordingPoseSourceLabel}`}</strong>
                  </div>
                </div>
                <div className="area-action-grid">
                  {!recordingInProgress ? (
                    <button
                      className="command-button"
                      disabled={areaActionDisabled(ACTION_START_RECORDING)}
                      type="button"
                      onClick={() => callAreaAction(ACTION_START_RECORDING)}
                    >
                      <CircleDot size={18} aria-hidden="true" />
                      <span>Start Recording</span>
                    </button>
                  ) : (
                    <button
                      className="command-button is-danger"
                      disabled={areaActionDisabled(ACTION_STOP_RECORDING)}
                      type="button"
                      onClick={() => callAreaAction(ACTION_STOP_RECORDING)}
                    >
                      <Square size={18} aria-hidden="true" />
                      <span>Stop Recording</span>
                    </button>
                  )}
                  <button
                    className="command-button"
                    disabled={!connected || !hasAnyFinishAction}
                    type="button"
                    onClick={() => setFinishAreaDialogOpen(true)}
                  >
                    <Save size={18} aria-hidden="true" />
                    <span>Finish Area</span>
                  </button>
                  <button
                    className="command-button"
                    disabled={areaActionDisabled(autoCollectingActive ? ACTION_AUTO_COLLECT_DISABLE : ACTION_AUTO_COLLECT_ENABLE)}
                    type="button"
                    onClick={() => callAreaAction(autoCollectingActive ? ACTION_AUTO_COLLECT_DISABLE : ACTION_AUTO_COLLECT_ENABLE)}
                  >
                    <Route size={18} aria-hidden="true" />
                    <span>{autoCollectingActive ? "Disable Auto" : "Enable Auto"}</span>
                  </button>
                  <button
                    className="command-button"
                    disabled={areaActionDisabled(ACTION_COLLECT_POINT)}
                    type="button"
                    onClick={() => callAreaAction(ACTION_COLLECT_POINT)}
                  >
                    <Plus size={18} aria-hidden="true" />
                    <span>Add Point</span>
                  </button>
                  <button
                    className="command-button is-secondary"
                    disabled={areaActionDisabled(ACTION_EXIT_RECORDING_MODE)}
                    type="button"
                    onClick={() => callAreaAction(ACTION_EXIT_RECORDING_MODE)}
                  >
                    <LogOut size={18} aria-hidden="true" />
                    <span>Exit Mode</span>
                  </button>
                </div>
              </div>

              <div className="area-recording-actions">
                <div className="area-recording-panel-header">
                  <div>
                    <span className="eyebrow">Teach Path</span>
                    <strong>{manualPathStatusLabel}</strong>
                  </div>
                </div>
                <label className="layer-toggle manual-path-raw-toggle" title="Capture raw ROS bag during path capture">
                  <input
                    type="checkbox"
                    checked={manualPathRawBag}
                    disabled={manualPathActive || manualPathPendingAction !== null}
                    onChange={(event) => setManualPathRawBag(event.target.checked)}
                  />
                  <span>Raw Bag</span>
                </label>
                <div className="area-recording-mini-status manual-path-mini-status">
                  <div>
                    <span>Samples</span>
                    <strong>{manualPathSyncSamples}</strong>
                  </div>
                  <div>
                    <span>Rejects</span>
                    <strong>{manualPathRejectCount}</strong>
                  </div>
                </div>
                <div className="area-action-grid manual-path-action-grid">
                  <button
                    className="command-button"
                    disabled={!connected || !areaRecordingActive || !manualPathOnline || manualPathActive || manualPathPendingAction !== null}
                    type="button"
                    onClick={() => void startManualPathCapture()}
                  >
                    <Play size={18} aria-hidden="true" />
                    <span>{manualPathPendingAction === "start" ? "Starting" : "Start Path Capture"}</span>
                  </button>
                  <button
                    className="command-button is-danger"
                    disabled={!connected || !manualPathOnline || !manualPathActive || manualPathPendingAction !== null}
                    type="button"
                    onClick={() => void callManualPathRecorderAction("stop")}
                  >
                    <Square size={18} aria-hidden="true" />
                    <span>{manualPathPendingAction === "stop" ? "Stopping" : "Stop Path Capture"}</span>
                  </button>
                  <button
                    className="command-button"
                    disabled={!connected || !manualPathOnline || !manualPathActive || manualPathPendingAction !== null}
                    type="button"
                    onClick={() => void callManualPathRecorderAction("mark")}
                  >
                    <Flag size={18} aria-hidden="true" />
                    <span>{manualPathPendingAction === "mark" ? "Marking" : "Mark Event"}</span>
                  </button>
                  <button
                    className="command-button is-secondary"
                    disabled={!connected || !manualPathOnline || manualPathActive || !manualPathSessionAvailable || manualPathPendingAction !== null}
                    type="button"
                    onClick={() => void callManualPathRecorderAction("export")}
                  >
                    <Upload size={18} aria-hidden="true" />
                    <span>{manualPathPendingAction === "export" ? "Exporting" : "Export"}</span>
                  </button>
                </div>
                {manualPathRecorderStatus?.last_error && <div className="status-banner">{manualPathRecorderStatus.last_error}</div>}
              </div>
            </div>
          )}
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
              <span>Final</span>
              <strong>{metricValues.fusionState}</strong>
            </div>
            <div>
              <span>Anchor</span>
              <strong>{metricValues.anchor}</strong>
            </div>
          </div>
        </div>
      </section>

      {mapNameDialog === "create" && (
        <div className="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="create-map-title">
          <form className="warning-modal map-name-modal" onSubmit={createMap}>
            <div className="warning-modal-header">
              <Plus size={22} aria-hidden="true" />
              <h2 id="create-map-title">Create Map</h2>
              <button aria-label="Cancel create map" className="icon-button" type="button" onClick={() => setMapNameDialog(null)}>
                <X size={18} aria-hidden="true" />
              </button>
            </div>
            <label className="map-name-field" htmlFor="dialog-new-map-name">
              <span>Map name</span>
              <input
                aria-label="New map name"
                disabled={!mapMutationAllowed || mapCommandPending}
                id="dialog-new-map-name"
                placeholder="New map name"
                ref={newMapNameInputRef}
                type="text"
                value={newMapName}
                onChange={(event) => setNewMapName(event.target.value)}
              />
            </label>
            {!mapMutationAllowed && <div className="status-banner">Map edits require IDLE state</div>}
            {mapCommandMessage && <div className="status-banner">{mapCommandMessage}</div>}
            <div className="modal-actions">
              <button className="command-button is-secondary" type="button" onClick={() => setMapNameDialog(null)}>
                <span>Cancel</span>
              </button>
              <button
                className="command-button"
                disabled={!mapMutationAllowed || mapCommandPending || !newMapName.trim()}
                type="submit"
              >
                <span>{mapCommandPending ? "Creating" : "Create"}</span>
              </button>
            </div>
          </form>
        </div>
      )}

      {mapNameDialog === "rename" && (
        <div className="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="rename-map-title">
          <form className="warning-modal map-name-modal" onSubmit={renameSelectedMap}>
            <div className="warning-modal-header">
              <Edit3 size={22} aria-hidden="true" />
              <h2 id="rename-map-title">Rename Map</h2>
              <button aria-label="Cancel rename map" className="icon-button" type="button" onClick={() => setMapNameDialog(null)}>
                <X size={18} aria-hidden="true" />
              </button>
            </div>
            <label className="map-name-field" htmlFor="dialog-rename-map-name">
              <span>Map name</span>
              <input
                aria-label="Selected map name"
                disabled={!mapMutationAllowed || mapCommandPending || !selectedMap}
                id="dialog-rename-map-name"
                placeholder="Selected map name"
                ref={renameMapNameInputRef}
                type="text"
                value={renameMapName}
                onChange={(event) => setRenameMapName(event.target.value)}
              />
            </label>
            {!mapMutationAllowed && <div className="status-banner">Map edits require IDLE state</div>}
            {mapCommandMessage && <div className="status-banner">{mapCommandMessage}</div>}
            <div className="modal-actions">
              <button className="command-button is-secondary" type="button" onClick={() => setMapNameDialog(null)}>
                <span>Cancel</span>
              </button>
              <button
                className="command-button"
                disabled={!mapMutationAllowed || mapCommandPending || !selectedMap || !renameMapName.trim()}
                type="submit"
              >
                <span>{mapCommandPending ? "Renaming" : "Rename"}</span>
              </button>
            </div>
          </form>
        </div>
      )}

      {finishAreaDialogOpen && (
        <div className="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="finish-area-title">
          <div className="warning-modal area-finish-modal">
            <div className="warning-modal-header">
              <Save size={22} aria-hidden="true" />
              <h2 id="finish-area-title">Save Area</h2>
              <button aria-label="Cancel save area" className="icon-button" type="button" onClick={() => setFinishAreaDialogOpen(false)}>
                <X size={18} aria-hidden="true" />
              </button>
            </div>
            <div className="modal-actions area-finish-actions">
              <button
                className="command-button"
                disabled={areaActionDisabled(ACTION_FINISH_MOWING_AREA)}
                type="button"
                onClick={() => {
                  callAreaAction(ACTION_FINISH_MOWING_AREA);
                  setFinishAreaDialogOpen(false);
                }}
              >
                <span>Mowing Area</span>
              </button>
              <button
                className="command-button"
                disabled={areaActionDisabled(ACTION_FINISH_NAVIGATION_AREA)}
                type="button"
                onClick={() => {
                  callAreaAction(ACTION_FINISH_NAVIGATION_AREA);
                  setFinishAreaDialogOpen(false);
                }}
              >
                <span>Navigation Area</span>
              </button>
              <button
                className="command-button is-danger"
                disabled={areaActionDisabled(ACTION_FINISH_DISCARD)}
                type="button"
                onClick={() => {
                  callAreaAction(ACTION_FINISH_DISCARD);
                  setFinishAreaDialogOpen(false);
                }}
              >
                <span>Don&apos;t Save</span>
              </button>
            </div>
          </div>
        </div>
      )}

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
