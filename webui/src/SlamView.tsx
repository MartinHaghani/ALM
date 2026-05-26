import { Activity, Crosshair, Pause, Play, RefreshCw, Square, Trash2 } from "lucide-react";
import { FormEvent, useMemo, useState } from "react";
import type { Ros } from "roslib";

import type { NextWebUiConfig } from "./config";
import { callSetBoolService, callTriggerService } from "./rosServices";
import { SlamMapCanvas } from "./SlamMapCanvas";
import { lookupTransform2D, type Transform2D } from "./tfMath";
import type { AbsolutePose, LaserScan, NavSatFix } from "./types";
import { useGpsFix } from "./useGpsFix";
import { useGpsStatus } from "./useGpsStatus";
import { useLaserScan } from "./useLaserScan";
import { useOccupancyGrid } from "./useOccupancyGrid";
import { useSlamManagerStatus } from "./useSlamManagerStatus";
import { useTfFrames } from "./useTfFrames";

const GPS_STALE_MS = 3000;
const TF_STALE_MS = 2500;
const FLAG_GPS_RTK_FIXED = 2;
const FLAG_GPS_RTK_FLOAT = 4;
const FLAG_GPS_DEAD_RECKONING = 8;

interface SlamViewProps {
  config: NextWebUiConfig;
  connected: boolean;
  error: string | null;
  now: number;
  ros: Ros | null;
  url: string;
}

function formatNumber(value: number, digits = 1): string {
  if (!Number.isFinite(value)) {
    return "--";
  }
  return value.toFixed(digits);
}

function formatAge(lastMessageAt: number | null, now: number): string {
  if (!lastMessageAt) {
    return "--";
  }
  return `${formatNumber(Math.max(0, (now - lastMessageAt) / 1000), 1)} s`;
}

function gpsLabel(connected: boolean, fix: NavSatFix | null, status: AbsolutePose | null, lastFixAt: number | null, now: number): string {
  if (!connected) {
    return "ROS offline";
  }
  if (!fix || !lastFixAt) {
    return "Waiting";
  }
  if (now - lastFixAt > GPS_STALE_MS) {
    return "Stale";
  }
  if (fix.status.status < 0) {
    return "No fix";
  }

  const flags = status?.flags ?? 0;
  if ((flags & FLAG_GPS_RTK_FIXED) !== 0) {
    return "RTK fixed";
  }
  if ((flags & FLAG_GPS_RTK_FLOAT) !== 0) {
    return "RTK float";
  }
  if ((flags & FLAG_GPS_DEAD_RECKONING) !== 0) {
    return "Dead reckoning";
  }
  return "GPS fix";
}

function finiteRanges(scan: LaserScan | null): number[] {
  if (!scan) {
    return [];
  }
  return scan.ranges.filter((range) => Number.isFinite(range) && range >= scan.range_min && range <= scan.range_max);
}

function tfState(transform: Transform2D | null): "ok" | "missing" {
  return transform ? "ok" : "missing";
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

export function SlamView({ config, connected, error, now, ros, url }: SlamViewProps) {
  const [scanTopic, setScanTopic] = useState(config.slamScanTopic);
  const [draftScanTopic, setDraftScanTopic] = useState(config.slamScanTopic);
  const [mapTopic, setMapTopic] = useState(config.slamMapTopic);
  const [draftMapTopic, setDraftMapTopic] = useState(config.slamMapTopic);
  const [paused, setPaused] = useState(false);
  const [clampMeters, setClampMeters] = useState(12);
  const [commandMessage, setCommandMessage] = useState<string | null>(null);
  const [commandPending, setCommandPending] = useState(false);
  const [mapOpacity, setMapOpacity] = useState(0.92);
  const [mapResetKey, setMapResetKey] = useState(0);
  const [zoom, setZoom] = useState(1);

  const { grid, stats: mapStats } = useOccupancyGrid({
    paused,
    resetKey: mapResetKey,
    ros,
    topicName: mapTopic,
  });
  const { scan, stats: scanStats } = useLaserScan({
    paused,
    ros,
    throttleMs: 100,
    topicName: scanTopic,
  });
  const { stats: tfStats, transforms } = useTfFrames({
    ros,
    tfStaticTopic: config.tfStaticTopic,
    tfTopic: config.tfTopic,
  });
  const { fix, stats: fixStats } = useGpsFix({
    ros,
    topicName: config.gpsFixTopic,
  });
  const { stats: statusStats, status } = useGpsStatus({
    ros,
    topicName: config.gpsStatusTopic,
  });
  const { stats: managerStats, status: managerStatus } = useSlamManagerStatus({
    ros,
    topicName: config.slamManagerStatusTopic,
  });
  const tfLookupOptions = useMemo(() => ({ maxAgeMs: TF_STALE_MS, nowMs: now }), [now]);

  const scanFrame = scan?.header.frame_id || "lidar";
  const slamToOdom = useMemo(
    () => lookupTransform2D(transforms, config.slamMapFrame, config.slamOdomFrame, tfLookupOptions),
    [config.slamMapFrame, config.slamOdomFrame, tfLookupOptions, transforms],
  );
  const odomToBase = useMemo(
    () => lookupTransform2D(transforms, config.slamOdomFrame, config.slamBaseFrame, tfLookupOptions),
    [config.slamBaseFrame, config.slamOdomFrame, tfLookupOptions, transforms],
  );
  const baseToScan = useMemo(
    () => lookupTransform2D(transforms, config.slamBaseFrame, scanFrame, tfLookupOptions),
    [config.slamBaseFrame, scanFrame, tfLookupOptions, transforms],
  );
  const mapToOperationalBase = useMemo(
    () => lookupTransform2D(transforms, "map", "base_link", tfLookupOptions),
    [tfLookupOptions, transforms],
  );
  const operationalBaseToRawLidar = useMemo(
    () => lookupTransform2D(transforms, "base_link", "lidar", tfLookupOptions),
    [tfLookupOptions, transforms],
  );
  const slamToBase = useMemo(
    () => lookupTransform2D(transforms, config.slamMapFrame, config.slamBaseFrame, tfLookupOptions),
    [config.slamBaseFrame, config.slamMapFrame, tfLookupOptions, transforms],
  );
  const slamToLidar = useMemo(
    () => lookupTransform2D(transforms, config.slamMapFrame, scanFrame, tfLookupOptions),
    [config.slamMapFrame, scanFrame, tfLookupOptions, transforms],
  );

  const ranges = finiteRanges(scan);
  const minRange = ranges.length > 0 ? Math.min(...ranges) : Number.NaN;
  const maxRange = ranges.length > 0 ? Math.max(...ranges) : Number.NaN;
  const statusIsFresh = statusStats.lastMessageAt !== null && now - statusStats.lastMessageAt <= GPS_STALE_MS;
  const freshStatus = statusIsFresh ? status : null;
  const managerIsFresh = managerStats.lastMessageAt !== null && now - managerStats.lastMessageAt <= GPS_STALE_MS;
  const mappingEnabled = managerIsFresh && Boolean(managerStatus?.mapping_enabled);
  const slamRunning = managerIsFresh && Boolean(managerStatus?.slam_running);
  const slamState = managerLabel(connected, managerIsFresh, mappingEnabled, slamRunning);
  const gridWidthMeters = grid ? grid.info.width * grid.info.resolution : Number.NaN;
  const gridHeightMeters = grid ? grid.info.height * grid.info.resolution : Number.NaN;

  function applyScanTopic(event: FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    const nextTopic = draftScanTopic.trim();
    if (nextTopic) {
      setScanTopic(nextTopic);
    }
  }

  function applyMapTopic(event: FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    const nextTopic = draftMapTopic.trim();
    if (nextTopic) {
      setMapTopic(nextTopic);
    }
  }

  async function toggleMapping(): Promise<void> {
    if (!ros || !connected || commandPending) {
      return;
    }

    const nextEnabled = !mappingEnabled;
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

  return (
    <div className="workspace slam-workspace">
      <aside className="control-panel">
        <section className="panel-section">
          <h1>Passive SLAM</h1>
          <p className="subtle">{url}</p>
        </section>

        <div className={`sensor-pill is-${connected ? "functioning" : "offline"}`}>
          <Activity size={18} aria-hidden="true" />
          <span>{connected ? "ROS connected" : "ROS offline"}</span>
        </div>

        <section className="panel-section">
          <div className={`sensor-pill is-${mappingEnabled ? "functioning" : "waiting"}`}>
            <Activity size={18} aria-hidden="true" />
            <span>{slamState}</span>
          </div>
          <div className="control-row control-row-split">
            <button className="command-button" type="button" disabled={!connected || commandPending} onClick={toggleMapping}>
              {mappingEnabled ? <Square size={18} aria-hidden="true" /> : <Play size={18} aria-hidden="true" />}
              <span>{commandPending ? "Working" : mappingEnabled ? "Stop mapping" : "Start mapping"}</span>
            </button>
            <button className="command-button is-danger" type="button" disabled={!connected || commandPending} onClick={clearSlamMap}>
              <Trash2 size={18} aria-hidden="true" />
              <span>Clear map</span>
            </button>
          </div>
          {commandMessage && <div className="status-banner">{commandMessage}</div>}
          {managerStatus?.last_error && <div className="error-banner">{managerStatus.last_error}</div>}
        </section>

        <form className="topic-form" onSubmit={applyMapTopic}>
          <label htmlFor="slam-map-topic">Map topic</label>
          <div className="topic-row">
            <input
              id="slam-map-topic"
              value={draftMapTopic}
              onChange={(event) => setDraftMapTopic(event.target.value)}
              spellCheck={false}
            />
            <button className="icon-button" type="submit" aria-label="Apply SLAM map topic">
              <RefreshCw size={18} aria-hidden="true" />
            </button>
          </div>
        </form>

        <form className="topic-form" onSubmit={applyScanTopic}>
          <label htmlFor="slam-scan-topic">Scan topic</label>
          <div className="topic-row">
            <input
              id="slam-scan-topic"
              value={draftScanTopic}
              onChange={(event) => setDraftScanTopic(event.target.value)}
              spellCheck={false}
            />
            <button className="icon-button" type="submit" aria-label="Apply SLAM scan topic">
              <RefreshCw size={18} aria-hidden="true" />
            </button>
          </div>
        </form>

        <section className="panel-section">
          <div className="control-row">
            <button className="command-button" type="button" onClick={() => setPaused((current) => !current)}>
              {paused ? <Play size={18} aria-hidden="true" /> : <Pause size={18} aria-hidden="true" />}
              <span>{paused ? "Resume" : "Pause"}</span>
            </button>
          </div>
          <label htmlFor="slam-range-clamp">Range clamp</label>
          <div className="range-row">
            <input
              id="slam-range-clamp"
              min="1"
              max="18"
              step="0.5"
              type="range"
              value={clampMeters}
              onChange={(event) => setClampMeters(Number(event.target.value))}
            />
            <output htmlFor="slam-range-clamp">{formatNumber(clampMeters, 1)} m</output>
          </div>
          <label htmlFor="slam-map-opacity">Map opacity</label>
          <div className="range-row">
            <input
              id="slam-map-opacity"
              min="0.25"
              max="1"
              step="0.05"
              type="range"
              value={mapOpacity}
              onChange={(event) => setMapOpacity(Number(event.target.value))}
            />
            <output htmlFor="slam-map-opacity">{Math.round(mapOpacity * 100)}%</output>
          </div>
          <label htmlFor="slam-zoom">Map zoom</label>
          <div className="range-row">
            <input
              id="slam-zoom"
              min="1"
              max="4"
              step="0.25"
              type="range"
              value={zoom}
              onChange={(event) => setZoom(Number(event.target.value))}
            />
            <output htmlFor="slam-zoom">{formatNumber(zoom, 2)}x</output>
          </div>
          <button className="command-button" type="button" onClick={() => setZoom(1)}>
            <Crosshair size={18} aria-hidden="true" />
            <span>Fit</span>
          </button>
        </section>

        <section className="metric-grid" aria-label="SLAM metrics">
          <div className="metric">
            <span>Map age</span>
            <strong>{formatAge(mapStats.lastMessageAt, now)}</strong>
          </div>
          <div className="metric">
            <span>Scan Hz</span>
            <strong>{formatNumber(scanStats.hz, 1)}</strong>
          </div>
          <div className="metric">
            <span>Cells</span>
            <strong>{grid ? `${grid.info.width} x ${grid.info.height}` : "--"}</strong>
          </div>
          <div className="metric">
            <span>Meters</span>
            <strong>
              {Number.isFinite(gridWidthMeters) ? `${formatNumber(gridWidthMeters, 1)} x ${formatNumber(gridHeightMeters, 1)}` : "--"}
            </strong>
          </div>
          <div className="metric">
            <span>Samples</span>
            <strong>{scan?.ranges.length ?? 0}</strong>
          </div>
          <div className="metric">
            <span>RTK</span>
            <strong>{gpsLabel(connected, fix, freshStatus, fixStats.lastMessageAt, now)}</strong>
          </div>
        </section>

        {error && <div className="error-banner">{error}</div>}
      </aside>

      <main className="slam-stage">
        <section className="scan-stage slam-map-stage">
          <div className="scan-stage-header">
            <div>
              <span className="eyebrow">Shadow occupancy grid</span>
              <h2>{mapTopic}</h2>
            </div>
            <div className={`scan-state ${paused ? "is-paused" : ""}`}>
              <Activity size={18} aria-hidden="true" />
              <span>{paused ? "Paused" : slamState}</span>
            </div>
          </div>

          <SlamMapCanvas
            clampMeters={clampMeters}
            grid={grid}
            mapOpacity={mapOpacity}
            scan={scan}
            slamToBase={slamToBase}
            slamToLidar={slamToLidar}
            zoom={zoom}
          />

          <div className="slam-status-strip" aria-label="SLAM transform and scan status">
            <div className={`tf-chip is-${tfState(slamToOdom)}`}>
              <span>{`${config.slamMapFrame} -> ${config.slamOdomFrame}`}</span>
              <strong>{tfState(slamToOdom)}</strong>
            </div>
            <div className={`tf-chip is-${tfState(odomToBase)}`}>
              <span>{`${config.slamOdomFrame} -> ${config.slamBaseFrame}`}</span>
              <strong>{tfState(odomToBase)}</strong>
            </div>
            <div className={`tf-chip is-${tfState(baseToScan)}`}>
              <span>{`${config.slamBaseFrame} -> ${scanFrame}`}</span>
              <strong>{tfState(baseToScan)}</strong>
            </div>
            <div className={`tf-chip is-${tfState(mapToOperationalBase)}`}>
              <span>{"map -> base_link"}</span>
              <strong>{tfState(mapToOperationalBase)}</strong>
            </div>
            <div className={`tf-chip is-${tfState(operationalBaseToRawLidar)}`}>
              <span>{"base_link -> lidar"}</span>
              <strong>{tfState(operationalBaseToRawLidar)}</strong>
            </div>
            <div>
              <span>TF Hz</span>
              <strong>{formatNumber(tfStats.hz, 1)}</strong>
            </div>
            <div>
              <span>Range</span>
              <strong>{formatNumber(minRange, 2)} - {formatNumber(maxRange, 2)} m</strong>
            </div>
          </div>
        </section>
      </main>
    </div>
  );
}
