import { Activity, Map as MapIcon, Pause, Play, Radar, RefreshCw, Wifi, WifiOff } from "lucide-react";
import { FormEvent, useEffect, useMemo, useState } from "react";
import type { Ros } from "roslib";

import { CombinedMapView } from "./CombinedMapView";
import { ImuPanel } from "./ImuPanel";
import { ScanCanvas } from "./ScanCanvas";
import { getNextWebUiConfig, type NextWebUiConfig } from "./config";
import { isImuSampleValid } from "./imuMath";
import type { Imu, ImuStats, LaserScan } from "./types";
import { useImu } from "./useImu";
import { useLaserScan } from "./useLaserScan";
import { useRosBridge } from "./useRosBridge";

const DEFAULT_IMU_TOPIC = "/hw/imu/data_raw";
const IMU_STALE_MS = 1500;

type ImuStatus = "functioning" | "offline" | "waiting";
type ViewMode = "map" | "sensors";

interface SensorViewerProps {
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

function finiteRanges(scan: LaserScan | null): number[] {
  if (!scan) {
    return [];
  }
  return scan.ranges.filter((range) => Number.isFinite(range) && range >= scan.range_min && range <= scan.range_max);
}

function getImuStatus(connected: boolean, imu: Imu | null, stats: ImuStats, now: number): ImuStatus {
  if (!connected) {
    return "offline";
  }

  const isFresh = stats.lastMessageAt !== null && now - stats.lastMessageAt <= IMU_STALE_MS;
  if (isFresh && isImuSampleValid(imu)) {
    return "functioning";
  }

  return "waiting";
}

function imuStatusLabel(status: ImuStatus): string {
  if (status === "functioning") {
    return "IMU functioning";
  }
  if (status === "offline") {
    return "IMU offline";
  }
  return "IMU waiting";
}

function SensorViewer({ config, connected, error, now, ros, url }: SensorViewerProps) {
  const [scanTopic, setScanTopic] = useState(config.scanTopic);
  const [draftTopic, setDraftTopic] = useState(config.scanTopic);
  const [imuTopic, setImuTopic] = useState(DEFAULT_IMU_TOPIC);
  const [draftImuTopic, setDraftImuTopic] = useState(DEFAULT_IMU_TOPIC);
  const [paused, setPaused] = useState(false);
  const [clampMeters, setClampMeters] = useState(8);

  const { scan, stats } = useLaserScan({
    paused,
    ros,
    topicName: scanTopic,
  });
  const { imu, stats: imuStats } = useImu({
    ros,
    topicName: imuTopic,
  });

  const ranges = finiteRanges(scan);
  const minRange = ranges.length > 0 ? Math.min(...ranges) : Number.NaN;
  const maxRange = ranges.length > 0 ? Math.max(...ranges) : Number.NaN;
  const imuStatus = getImuStatus(connected, imu, imuStats, now);

  function applyTopic(event: FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    const nextTopic = draftTopic.trim();
    if (nextTopic) {
      setScanTopic(nextTopic);
    }
  }

  function applyImuTopic(event: FormEvent<HTMLFormElement>): void {
    event.preventDefault();
    const nextTopic = draftImuTopic.trim();
    if (nextTopic) {
      setImuTopic(nextTopic);
    }
  }

  return (
    <div className="workspace">
      <aside className="control-panel">
        <section className="panel-section">
          <h1>Sensor Viewer</h1>
          <p className="subtle">{url}</p>
        </section>

        <div className={`sensor-pill is-${imuStatus}`}>
          <Activity size={18} aria-hidden="true" />
          <span>{imuStatusLabel(imuStatus)}</span>
        </div>

        <form className="topic-form" onSubmit={applyTopic}>
          <label htmlFor="scan-topic">Scan topic</label>
          <div className="topic-row">
            <input
              id="scan-topic"
              value={draftTopic}
              onChange={(event) => setDraftTopic(event.target.value)}
              spellCheck={false}
            />
            <button className="icon-button" type="submit" aria-label="Apply scan topic">
              <RefreshCw size={18} aria-hidden="true" />
            </button>
          </div>
        </form>

        <form className="topic-form" onSubmit={applyImuTopic}>
          <label htmlFor="imu-topic">IMU topic</label>
          <div className="topic-row">
            <input
              id="imu-topic"
              value={draftImuTopic}
              onChange={(event) => setDraftImuTopic(event.target.value)}
              spellCheck={false}
            />
            <button className="icon-button" type="submit" aria-label="Apply IMU topic">
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
          <label htmlFor="range-clamp">Range clamp</label>
          <div className="range-row">
            <input
              id="range-clamp"
              min="1"
              max="12"
              step="0.5"
              type="range"
              value={clampMeters}
              onChange={(event) => setClampMeters(Number(event.target.value))}
            />
            <output htmlFor="range-clamp">{formatNumber(clampMeters, 1)} m</output>
          </div>
        </section>

        <section className="metric-grid" aria-label="Scan metrics">
          <div className="metric">
            <span>Hz</span>
            <strong>{formatNumber(stats.hz, 1)}</strong>
          </div>
          <div className="metric">
            <span>Age</span>
            <strong>{formatAge(stats.lastMessageAt, now)}</strong>
          </div>
          <div className="metric">
            <span>Samples</span>
            <strong>{scan?.ranges.length ?? 0}</strong>
          </div>
          <div className="metric">
            <span>Min</span>
            <strong>{formatNumber(minRange, 2)} m</strong>
          </div>
          <div className="metric">
            <span>Max</span>
            <strong>{formatNumber(maxRange, 2)} m</strong>
          </div>
          <div className="metric">
            <span>Frames</span>
            <strong>{stats.messageCount}</strong>
          </div>
        </section>

        {error && <div className="error-banner">{error}</div>}
      </aside>

      <main className="sensor-stage">
        <ImuPanel imu={imu} now={now} stats={imuStats} />

        <section className="scan-stage">
          <div className="scan-stage-header">
            <div>
              <span className="eyebrow">Live scan</span>
              <h2>{scanTopic}</h2>
            </div>
            <div className={`scan-state ${paused ? "is-paused" : ""}`}>
              <Activity size={18} aria-hidden="true" />
              <span>{paused ? "Paused" : "Streaming"}</span>
            </div>
          </div>
          <ScanCanvas scan={scan} clampMeters={clampMeters} />
        </section>
      </main>
    </div>
  );
}

export default function App() {
  const [viewMode, setViewMode] = useState<ViewMode>("map");
  const [now, setNow] = useState(Date.now());
  const config = useMemo(getNextWebUiConfig, []);

  const { connected, error, ros, url } = useRosBridge();

  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 250);
    return () => window.clearInterval(timer);
  }, []);

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand-mark">
          <Radar size={24} aria-hidden="true" />
          <span>Open Mower Next</span>
        </div>
        <div className="topbar-actions">
          <div className="view-switch" aria-label="View selector">
            <button
              className={viewMode === "map" ? "is-active" : ""}
              type="button"
              onClick={() => setViewMode("map")}
            >
              <MapIcon size={17} aria-hidden="true" />
              <span>Map</span>
            </button>
            <button
              className={viewMode === "sensors" ? "is-active" : ""}
              type="button"
              onClick={() => setViewMode("sensors")}
            >
              <Activity size={17} aria-hidden="true" />
              <span>Sensors</span>
            </button>
          </div>
          <div className={`connection-pill ${connected ? "is-connected" : "is-offline"}`}>
            {connected ? <Wifi size={18} aria-hidden="true" /> : <WifiOff size={18} aria-hidden="true" />}
            <span>{connected ? "ROS connected" : "ROS offline"}</span>
          </div>
        </div>
      </header>

      {viewMode === "map" && (
        <CombinedMapView config={config} connected={connected} error={error} now={now} ros={ros} url={url} />
      )}
      {viewMode === "sensors" && (
        <SensorViewer config={config} connected={connected} error={error} now={now} ros={ros} url={url} />
      )}
    </div>
  );
}
