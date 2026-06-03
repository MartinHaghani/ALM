import { Activity, AlertTriangle, Map as MapIcon, Pause, Play, Radar, RefreshCw, RotateCcw, Wifi, WifiOff } from "lucide-react";
import { FormEvent, useEffect, useMemo, useState } from "react";
import type { Ros } from "roslib";

import { CombinedMapView } from "./CombinedMapView";
import { ImuPanel } from "./ImuPanel";
import { ScanCanvas } from "./ScanCanvas";
import { getNextWebUiConfig, type NextWebUiConfig } from "./config";
import { isImuSampleValid } from "./imuMath";
import { callTriggerService } from "./rosServices";
import type { Imu, ImuStats, LaserScan } from "./types";
import { useImu } from "./useImu";
import { useLaserScan } from "./useLaserScan";
import { usePassiveSlamOdomStatus } from "./usePassiveSlamOdomStatus";
import { useRosBridge } from "./useRosBridge";

const DEFAULT_IMU_TOPIC = "/hw/imu/data_raw";
const IMU_STALE_MS = 1500;
const PASSIVE_ODOM_STALE_MS = 2000;

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

function formatPercent(value: number | null | undefined): string {
  if (value === null || value === undefined || !Number.isFinite(value)) {
    return "--";
  }
  return `${Math.round(Math.max(0, Math.min(1, value)) * 100)}%`;
}

function formatYawRate(value: number | null | undefined): string {
  if (value === null || value === undefined || !Number.isFinite(value)) {
    return "--";
  }
  return `${formatNumber((value * 180) / Math.PI, 1)} deg/s`;
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
  const [gyroCommandMessage, setGyroCommandMessage] = useState<string | null>(null);
  const [gyroCommandPending, setGyroCommandPending] = useState(false);

  const { scan, stats } = useLaserScan({
    paused,
    ros,
    topicName: scanTopic,
  });
  const { imu, stats: imuStats } = useImu({
    ros,
    topicName: imuTopic,
  });
  const { stats: passiveOdomStats, status: passiveOdomStatus } = usePassiveSlamOdomStatus({
    ros,
    topicName: config.passiveSlamOdomStatusTopic,
  });

  const ranges = finiteRanges(scan);
  const minRange = ranges.length > 0 ? Math.min(...ranges) : Number.NaN;
  const maxRange = ranges.length > 0 ? Math.max(...ranges) : Number.NaN;
  const imuStatus = getImuStatus(connected, imu, imuStats, now);
  const passiveOdomIsFresh =
    passiveOdomStats.lastMessageAt !== null && now - passiveOdomStats.lastMessageAt <= PASSIVE_ODOM_STALE_MS;
  const freshPassiveOdom = passiveOdomIsFresh ? passiveOdomStatus : null;
  const gyroWarning = Boolean(freshPassiveOdom?.gyro_stationary_warning);
  const gyroCalibrating = Boolean(freshPassiveOdom?.gyro_calibrating);

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

  async function calibrateGyro(): Promise<void> {
    if (!ros || !connected || gyroCommandPending) {
      return;
    }

    setGyroCommandPending(true);
    setGyroCommandMessage(null);
    try {
      const response = await callTriggerService(ros, config.passiveSlamGyroCalibrateService);
      setGyroCommandMessage(response.message || "Gyro calibration started");
      if (!response.success) {
        throw new Error(response.message || "Gyro calibration failed");
      }
    } catch (serviceError) {
      setGyroCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setGyroCommandPending(false);
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

        {gyroWarning && (
          <div className="warning-banner">
            <AlertTriangle size={16} aria-hidden="true" />
            <span>
              Gyro reports {formatYawRate(freshPassiveOdom?.yaw_rate)} while the mower appears stationary. Calibrate
              the gyro with the mower still.
            </span>
          </div>
        )}
        {gyroCalibrating && (
          <div className="status-banner">
            Gyro calibration running: {formatPercent(freshPassiveOdom?.gyro_calibration_progress)}
          </div>
        )}
        {gyroCommandMessage && <div className="status-banner">{gyroCommandMessage}</div>}

        <section className="panel-section">
          <div className="control-row">
            <button
              className="command-button"
              disabled={!connected || gyroCommandPending}
              type="button"
              onClick={calibrateGyro}
            >
              <RotateCcw size={18} aria-hidden="true" />
              <span>{gyroCommandPending ? "Starting" : "Calibrate gyro"}</span>
            </button>
          </div>
        </section>

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

        <section className="metric-grid" aria-label="Gyro diagnostics">
          <div className="metric">
            <span>Gyro rate</span>
            <strong>{formatYawRate(freshPassiveOdom?.yaw_rate)}</strong>
          </div>
          <div className="metric">
            <span>Raw gyro</span>
            <strong>{formatYawRate(freshPassiveOdom?.raw_yaw_rate)}</strong>
          </div>
          <div className="metric">
            <span>Offset</span>
            <strong>{formatYawRate(freshPassiveOdom?.gyro_offset)}</strong>
          </div>
          <div className="metric">
            <span>Still</span>
            <strong>{freshPassiveOdom?.stationary_detected === true ? "Yes" : freshPassiveOdom ? "No" : "--"}</strong>
          </div>
          <div className="metric">
            <span>Odom age</span>
            <strong>{formatAge(passiveOdomStats.lastMessageAt, now)}</strong>
          </div>
          <div className="metric">
            <span>Calibration</span>
            <strong>{gyroCalibrating ? formatPercent(freshPassiveOdom?.gyro_calibration_progress) : freshPassiveOdom?.calibrated ? "Done" : "--"}</strong>
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
