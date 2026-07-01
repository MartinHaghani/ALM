import {
  Activity,
  AlertTriangle,
  Battery,
  Bluetooth,
  Gamepad2,
  Link2,
  Map as MapIcon,
  Pause,
  Play,
  Power,
  Radar,
  RefreshCw,
  RotateCcw,
  Route,
  Search,
  Thermometer,
  Trash2,
  Unlink,
  Wifi,
  WifiOff,
  X,
} from "lucide-react";
import { FormEvent, useEffect, useMemo, useState } from "react";
import type { Ros } from "roslib";

import {
  BatteryVoltagePanel,
  batteryStateLabel,
  formatBatteryPercent,
  getBatteryPercent,
  getBatteryState,
  isHwPowerFresh,
  type BatteryVoltageLimits,
} from "./BatteryVoltagePanel";
import { CombinedMapView } from "./CombinedMapView";
import { ImuPanel } from "./ImuPanel";
import { ScanCanvas } from "./ScanCanvas";
import {
  TEMPERATURE_SENSORS,
  TemperaturePanel,
  formatTemperature,
  getOverallState,
  getTemperatureState,
  isReadingFresh,
  temperatureStateLabel,
} from "./TemperaturePanel";
import { getNextWebUiConfig, type NextWebUiConfig } from "./config";
import { isImuSampleValid } from "./imuMath";
import { callBluetoothDeviceCommandService, callSetBoolService, callSetManualInputSourceService, callTriggerService } from "./rosServices";
import type {
  BluetoothDevice,
  BluetoothGamepadStatus,
  HwPower,
  Imu,
  ImuStats,
  LaserScan,
  ManualInputSource,
  MowerInputStatus,
  SensorStats,
} from "./types";
import { useBluetoothGamepadStatus } from "./useBluetoothGamepadStatus";
import { useHwPower } from "./useHwPower";
import { useImu } from "./useImu";
import { useLaserScan } from "./useLaserScan";
import { useMowerInputStatus } from "./useMowerInputStatus";
import { usePassiveSlamOdomStatus } from "./usePassiveSlamOdomStatus";
import { useRosBridge } from "./useRosBridge";
import { useTemperatureSensors, type TemperatureReadings } from "./useTemperatureSensors";

const DEFAULT_IMU_TOPIC = "/hw/imu/data_raw";
const IMU_STALE_MS = 1500;
const PASSIVE_ODOM_STALE_MS = 2000;
const GYRO_CALIBRATION_MAX_RAW_YAW_RATE_RAD_S = 0.05;

type ImuStatus = "functioning" | "offline" | "waiting";
type ViewMode = "map" | "route" | "sensors";

const DEFAULT_CONTROLLER_PROFILES = ["xbox360", "switch_pro", "shield", "ps3", "steam_stick", "steam_touch"];
const CONTROLLER_PROFILE_LABELS: Record<string, string> = {
  ps3: "PS3",
  shield: "Shield",
  steam_stick: "Steam Stick",
  steam_touch: "Steam Touch",
  switch_pro: "Switch Pro",
  xbox360: "Xbox 360",
};

interface SensorViewerProps {
  batteryLimits: BatteryVoltageLimits;
  config: NextWebUiConfig;
  connected: boolean;
  error: string | null;
  now: number;
  power: HwPower | null;
  powerStats: SensorStats;
  ros: Ros | null;
  temperatureReadings: TemperatureReadings;
  url: string;
}

interface BluetoothControlPanelProps {
  bluetoothStatus: BluetoothGamepadStatus | null;
  config: NextWebUiConfig;
  connected: boolean;
  mowerInputStatus: MowerInputStatus | null;
  onClose: () => void;
  ros: Ros | null;
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

function controllerProfileLabel(profile: string): string {
  return CONTROLLER_PROFILE_LABELS[profile] ?? profile;
}

function formatRssi(value: number | null): string {
  if (value === null || !Number.isFinite(value)) {
    return "--";
  }
  return `${Math.round(value)} dBm`;
}

function hottestTemperature(readings: TemperatureReadings, connected: boolean, now: number): number | null {
  return TEMPERATURE_SENSORS.reduce<number | null>((hottest, spec) => {
    const reading = readings[spec.id];
    if (!isReadingFresh(connected, reading, now)) {
      return hottest;
    }

    const value = reading?.data?.data;
    if (typeof value !== "number" || !Number.isFinite(value)) {
      return hottest;
    }
    return hottest === null || value > hottest ? value : hottest;
  }, null);
}

function HeaderTemperatureStatus({
  connected,
  now,
  readings,
}: {
  connected: boolean;
  now: number;
  readings: TemperatureReadings;
}) {
  const states = TEMPERATURE_SENSORS.map((spec) => getTemperatureState(spec, readings[spec.id], connected, now));
  const state = getOverallState(states, connected);
  const label = temperatureStateLabel(state, connected);
  const hottest = hottestTemperature(readings, connected, now);
  const text = hottest === null ? label : `${label} ${formatTemperature(hottest)}°`;

  return (
    <div
      className={`header-status-pill is-${state}`}
      title={`Thermal health: ${text}`}
      aria-label={`Thermal health ${text}`}
    >
      <Thermometer size={18} aria-hidden="true" />
      <span>{text}</span>
    </div>
  );
}

function HeaderBatteryStatus({
  connected,
  limits,
  now,
  power,
  stats,
}: {
  connected: boolean;
  limits: BatteryVoltageLimits;
  now: number;
  power: HwPower | null;
  stats: SensorStats;
}) {
  const fresh = isHwPowerFresh(connected, power, stats, now);
  const state = getBatteryState(connected, limits, now, power, stats);
  const percent = getBatteryPercent(power, fresh);
  const percentText = formatBatteryPercent(percent);
  const label = batteryStateLabel(state, connected);
  const voltageText = fresh && power?.battery_voltage_valid ? `${formatNumber(power.v_battery, 1)} V` : label;

  return (
    <div
      className={`header-status-pill is-${state}`}
      title={`Battery: ${percentText}, ${voltageText}`}
      aria-label={`Battery ${percentText}, ${voltageText}`}
    >
      <Battery size={18} aria-hidden="true" />
      <span>{percentText}</span>
    </div>
  );
}

function deviceName(device: BluetoothDevice): string {
  return device.alias || device.name || "Controller";
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

function SensorViewer({
  batteryLimits,
  config,
  connected,
  error,
  now,
  power,
  powerStats,
  ros,
  temperatureReadings,
  url,
}: SensorViewerProps) {
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

    const rawYawRate = imu?.angular_velocity?.z;
    const imuIsFresh = imuStats.lastMessageAt !== null && now - imuStats.lastMessageAt <= IMU_STALE_MS;
    if (
      !imuIsFresh ||
      rawYawRate === null ||
      rawYawRate === undefined ||
      !Number.isFinite(rawYawRate) ||
      Math.abs(rawYawRate) > GYRO_CALIBRATION_MAX_RAW_YAW_RATE_RAD_S
    ) {
      setGyroCommandMessage(
        `Raw gyro is not quiet enough to calibrate: ${formatYawRate(rawYawRate)}. Keep the mower still and try again.`,
      );
      return;
    }

    setGyroCommandPending(true);
    setGyroCommandMessage(null);
    try {
      const calibrations = [
        { label: "Positioning", service: config.positioningGyroCalibrateService },
        { label: "Passive SLAM", service: config.passiveSlamGyroCalibrateService },
      ];
      const results = await Promise.allSettled(
        calibrations.map(async (calibration) => ({
          ...calibration,
          response: await callTriggerService(ros, calibration.service),
        })),
      );
      const messages: string[] = [];
      const failures: string[] = [];

      results.forEach((result, index) => {
        const label = calibrations[index].label;
        if (result.status === "rejected") {
          failures.push(`${label}: ${result.reason instanceof Error ? result.reason.message : String(result.reason)}`);
          return;
        }

        const message = result.value.response.message || "calibration started";
        messages.push(`${label}: ${message}`);
        if (!result.value.response.success) {
          failures.push(`${label}: ${message}`);
        }
      });

      setGyroCommandMessage(messages.join(" | ") || "Gyro calibration started");
      if (failures.length > 0) {
        throw new Error(failures.join(" | "));
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
        <TemperaturePanel connected={connected} now={now} readings={temperatureReadings} />

        <BatteryVoltagePanel
          connected={connected}
          limits={batteryLimits}
          now={now}
          power={power}
          stats={powerStats}
        />

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

function BluetoothControlPanel({
  bluetoothStatus,
  config,
  connected,
  mowerInputStatus,
  onClose,
  ros,
}: BluetoothControlPanelProps) {
  const [commandMessage, setCommandMessage] = useState<string | null>(null);
  const [pendingKey, setPendingKey] = useState<string | null>(null);
  const [selectedProfile, setSelectedProfile] = useState(bluetoothStatus?.controller_type ?? "xbox360");

  useEffect(() => {
    if (bluetoothStatus?.controller_type) {
      setSelectedProfile(bluetoothStatus.controller_type);
    }
  }, [bluetoothStatus?.controller_type]);

  const adapter = bluetoothStatus?.adapter ?? null;
  const devices = bluetoothStatus?.devices ?? [];
  const activeSource = mowerInputStatus?.active_source ?? "web_gamepad";
  const profileOptions = bluetoothStatus?.supported_profiles?.length
    ? bluetoothStatus.supported_profiles
    : DEFAULT_CONTROLLER_PROFILES;
  const canUseServices = Boolean(ros && connected);
  const directInputLive = Boolean(mowerInputStatus?.direct_connected);

  async function runCommand(key: string, command: () => Promise<{ message: string; success: boolean }>): Promise<void> {
    if (!canUseServices) {
      setCommandMessage("ROS offline");
      return;
    }

    setPendingKey(key);
    setCommandMessage(null);
    try {
      const response = await command();
      setCommandMessage(response.message || "Done");
      if (!response.success) {
        throw new Error(response.message || "Command failed");
      }
    } catch (serviceError) {
      setCommandMessage(serviceError instanceof Error ? serviceError.message : String(serviceError));
    } finally {
      setPendingKey(null);
    }
  }

  function setPowered(powered: boolean): void {
    void runCommand("power", () => callSetBoolService(ros as Ros, config.bluetoothPowerService, powered));
  }

  function setScanning(scanning: boolean): void {
    void runCommand("scan", () => callSetBoolService(ros as Ros, config.bluetoothScanService, scanning));
  }

  function setManualInputSource(source: ManualInputSource): void {
    void runCommand(`source-${source}`, () => callSetManualInputSourceService(ros as Ros, config.manualInputSetSourceService, source));
  }

  function runDeviceCommand(key: string, serviceName: string, device: BluetoothDevice): void {
    void runCommand(`${key}-${device.address}`, () =>
      callBluetoothDeviceCommandService(ros as Ros, serviceName, device.address, selectedProfile),
    );
  }

  return (
    <div className="bluetooth-popover" role="dialog" aria-label="Bluetooth controller">
      <div className="bluetooth-panel-header">
        <div>
          <span className="eyebrow">Controller</span>
          <h2>Bluetooth</h2>
        </div>
        <button className="icon-button" type="button" aria-label="Close Bluetooth panel" onClick={onClose}>
          <X size={18} aria-hidden="true" />
        </button>
      </div>

      <div className="bluetooth-adapter-row">
        <span className={`status-badge ${bluetoothStatus?.available ? "is-green" : "is-amber"}`}>
          {bluetoothStatus?.available ? "Adapter" : "Offline"}
        </span>
        <span className={`status-badge ${adapter?.powered ? "is-green" : "is-muted"}`}>
          {adapter?.powered ? "Powered" : "Off"}
        </span>
        <span className={`status-badge ${adapter?.discovering ? "is-blue" : "is-muted"}`}>
          {adapter?.discovering ? "Scanning" : "Idle"}
        </span>
        <span className={`status-badge ${bluetoothStatus?.agent_available ? "is-green" : "is-muted"}`}>
          Agent
        </span>
      </div>

      <div className="input-source-switch" aria-label="Manual input source">
        <button
          className={activeSource === "web_gamepad" ? "is-active" : ""}
          disabled={!canUseServices || pendingKey !== null}
          type="button"
          onClick={() => setManualInputSource("web_gamepad")}
        >
          Web Gamepad
        </button>
        <button
          className={activeSource === "direct_bluetooth" ? "is-active" : ""}
          disabled={!canUseServices || pendingKey !== null}
          type="button"
          onClick={() => setManualInputSource("direct_bluetooth")}
        >
          Direct Bluetooth
        </button>
      </div>

      <label className="bluetooth-profile-field" htmlFor="controller-profile">
        <span>Profile</span>
        <select
          id="controller-profile"
          value={selectedProfile}
          onChange={(event) => setSelectedProfile(event.target.value)}
        >
          {profileOptions.map((profile) => (
            <option key={profile} value={profile}>
              {controllerProfileLabel(profile)}
            </option>
          ))}
        </select>
      </label>

      <div className="bluetooth-command-row">
        <button
          className="command-button"
          disabled={!canUseServices || pendingKey !== null}
          type="button"
          onClick={() => setPowered(!adapter?.powered)}
        >
          <Power size={17} aria-hidden="true" />
          <span>{adapter?.powered ? "Power off" : "Power on"}</span>
        </button>
        <button
          className="command-button"
          disabled={!canUseServices || pendingKey !== null}
          type="button"
          onClick={() => setScanning(!adapter?.discovering)}
        >
          <Search size={17} aria-hidden="true" />
          <span>{adapter?.discovering ? "Stop scan" : "Scan"}</span>
        </button>
      </div>

      {(commandMessage || bluetoothStatus?.message) && (
        <div className="bluetooth-message">{commandMessage || bluetoothStatus?.message}</div>
      )}

      <div className="bluetooth-device-list">
        {devices.length === 0 ? (
          <div className="bluetooth-empty">No devices</div>
        ) : (
          devices.map((device) => {
            const inputLive = directInputLive && device.connected && device.services_resolved;

            return (
              <article className="bluetooth-device-card" key={device.address}>
                <div className="bluetooth-device-main">
                  <div>
                    <strong>{deviceName(device)}</strong>
                    <span>{device.address}</span>
                  </div>
                  <div className="bluetooth-rssi">{formatRssi(device.rssi)}</div>
                </div>
                <div className="bluetooth-badge-row">
                  {inputLive && <span className="status-badge is-green">Input live</span>}
                  {device.connected && !inputLive && (
                    <span className={`status-badge ${device.services_resolved ? "is-blue" : "is-amber"}`}>
                      {device.services_resolved ? "Link ready" : "Link only"}
                    </span>
                  )}
                  {device.paired && <span className="status-badge is-blue">Paired</span>}
                  {device.trusted && <span className="status-badge is-muted">Trusted</span>}
                </div>
                <div className="bluetooth-device-actions">
                  <button
                    className="icon-button"
                    disabled={!canUseServices || pendingKey !== null || device.paired}
                    title="Pair"
                    type="button"
                    onClick={() => runDeviceCommand("pair", config.bluetoothPairService, device)}
                  >
                    <Gamepad2 size={17} aria-hidden="true" />
                  </button>
                  <button
                    className="icon-button"
                    disabled={!canUseServices || pendingKey !== null || !device.paired || device.connected}
                    title="Connect"
                    type="button"
                    onClick={() => runDeviceCommand("connect", config.bluetoothConnectService, device)}
                  >
                    <Link2 size={17} aria-hidden="true" />
                  </button>
                  <button
                    className="icon-button"
                    disabled={!canUseServices || pendingKey !== null || !device.connected}
                    title="Disconnect"
                    type="button"
                    onClick={() => runDeviceCommand("disconnect", config.bluetoothDisconnectService, device)}
                  >
                    <Unlink size={17} aria-hidden="true" />
                  </button>
                  <button
                    className="icon-button is-danger"
                    disabled={!canUseServices || pendingKey !== null}
                    title="Forget"
                    type="button"
                    onClick={() => runDeviceCommand("forget", config.bluetoothForgetService, device)}
                  >
                    <Trash2 size={17} aria-hidden="true" />
                  </button>
                </div>
              </article>
            );
          })
        )}
      </div>
    </div>
  );
}

export default function App() {
  const [viewMode, setViewMode] = useState<ViewMode>("map");
  const [bluetoothOpen, setBluetoothOpen] = useState(false);
  const [now, setNow] = useState(Date.now());
  const config = useMemo(getNextWebUiConfig, []);

  const { connected, error, ros, url } = useRosBridge();
  const { status: bluetoothStatus } = useBluetoothGamepadStatus({
    ros,
    topicName: config.bluetoothStatusTopic,
  });
  const { status: mowerInputStatus } = useMowerInputStatus({
    ros,
    topicName: config.manualInputStatusTopic,
  });
  const temperatureReadings = useTemperatureSensors({
    ros,
    sensors: TEMPERATURE_SENSORS,
  });
  const { power, stats: powerStats } = useHwPower({
    ros,
    topicName: config.hwPowerTopic,
  });
  const batteryLimits = useMemo<BatteryVoltageLimits>(
    () => ({
      critical: config.batteryCriticalVoltage,
      empty: config.batteryEmptyVoltage,
      full: config.batteryFullVoltage,
      mismatchWarn: config.driveVoltageMismatchWarnV,
    }),
    [
      config.batteryCriticalVoltage,
      config.batteryEmptyVoltage,
      config.batteryFullVoltage,
      config.driveVoltageMismatchWarnV,
    ],
  );
  const bluetoothInputLive = Boolean(mowerInputStatus?.direct_connected);
  const bluetoothReady = Boolean(bluetoothStatus?.available && bluetoothStatus?.adapter?.powered);

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
              className={viewMode === "route" ? "is-active" : ""}
              type="button"
              onClick={() => setViewMode("route")}
            >
              <Route size={17} aria-hidden="true" />
              <span>Route</span>
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
          <div className="bluetooth-control">
            <button
              className={`bluetooth-header-button ${bluetoothInputLive ? "is-connected" : bluetoothReady ? "is-ready" : "is-offline"}`}
              title="Bluetooth controller"
              type="button"
              aria-label="Bluetooth controller"
              onClick={() => setBluetoothOpen((current) => !current)}
            >
              <Bluetooth size={19} aria-hidden="true" />
              <span className="bluetooth-header-dot" aria-hidden="true" />
            </button>
            {bluetoothOpen && (
              <BluetoothControlPanel
                bluetoothStatus={bluetoothStatus}
                config={config}
                connected={connected}
                mowerInputStatus={mowerInputStatus}
                onClose={() => setBluetoothOpen(false)}
                ros={ros}
              />
            )}
          </div>
          <HeaderTemperatureStatus connected={connected} now={now} readings={temperatureReadings} />
          <HeaderBatteryStatus
            connected={connected}
            limits={batteryLimits}
            now={now}
            power={power}
            stats={powerStats}
          />
          <div className={`connection-pill ${connected ? "is-connected" : "is-offline"}`}>
            {connected ? <Wifi size={18} aria-hidden="true" /> : <WifiOff size={18} aria-hidden="true" />}
            <span>{connected ? "ROS connected" : "ROS offline"}</span>
          </div>
        </div>
      </header>

      {viewMode === "map" && (
        <CombinedMapView
          config={config}
          connected={connected}
          error={error}
          mowerInputStatus={mowerInputStatus}
          now={now}
          ros={ros}
          url={url}
        />
      )}
      {viewMode === "route" && (
        <CombinedMapView
          config={config}
          connected={connected}
          error={error}
          mowerInputStatus={mowerInputStatus}
          now={now}
          routeMode
          ros={ros}
          url={url}
        />
      )}
      {viewMode === "sensors" && (
        <SensorViewer
          batteryLimits={batteryLimits}
          config={config}
          connected={connected}
          error={error}
          now={now}
          power={power}
          powerStats={powerStats}
          ros={ros}
          temperatureReadings={temperatureReadings}
          url={url}
        />
      )}
    </div>
  );
}
