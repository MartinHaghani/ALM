import type { CSSProperties } from "react";
import { AlertTriangle, Battery, Zap } from "lucide-react";

import type { HwPower, SensorStats } from "./types";

export type BatteryState = "good" | "low" | "critical" | "mismatch" | "offline";

export interface BatteryVoltageLimits {
  critical: number;
  empty: number;
  full: number;
  mismatchWarn: number;
}

interface BatteryVoltagePanelProps {
  connected: boolean;
  limits: BatteryVoltageLimits;
  now: number;
  power: HwPower | null;
  stats: SensorStats;
}

interface VoltageSource {
  label: string;
  detail: string;
  voltageKey: "left_drive_voltage" | "right_drive_voltage" | "mower_esc_voltage";
  validKey: "left_drive_voltage_valid" | "right_drive_voltage_valid" | "mower_esc_voltage_valid";
  usesDriveMismatch?: boolean;
}

const HW_POWER_STALE_MS = 3000;

const VOLTAGE_SOURCES: VoltageSource[] = [
  {
    label: "Drive L",
    detail: "75100 V2 Pro",
    voltageKey: "left_drive_voltage",
    validKey: "left_drive_voltage_valid",
    usesDriveMismatch: true,
  },
  {
    label: "Drive R",
    detail: "75100 V2 Pro",
    voltageKey: "right_drive_voltage",
    validKey: "right_drive_voltage_valid",
    usesDriveMismatch: true,
  },
  {
    label: "Blade ESC",
    detail: "75100 V2 Pro",
    voltageKey: "mower_esc_voltage",
    validKey: "mower_esc_voltage_valid",
  },
];

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function formatVoltage(value: number | null): string {
  if (value === null || !Number.isFinite(value)) {
    return "--";
  }
  return value.toFixed(1);
}

function formatAge(lastMessageAt: number | null, now: number): string {
  if (!lastMessageAt) {
    return "--";
  }
  return `${Math.max(0, (now - lastMessageAt) / 1000).toFixed(1)} s`;
}

export function formatBatteryPercent(value: number | null | undefined): string {
  if (value === null || value === undefined || !Number.isFinite(value)) {
    return "--";
  }
  return `${Math.round(clamp(value, 0, 1) * 100)}%`;
}

export function isHwPowerFresh(connected: boolean, power: HwPower | null, stats: SensorStats, now: number): boolean {
  return Boolean(connected && power && stats.lastMessageAt && now - stats.lastMessageAt <= HW_POWER_STALE_MS);
}

export function batteryStateLabel(state: BatteryState, connected: boolean): string {
  if (state === "offline" && !connected) {
    return "Offline";
  }
  if (state === "critical") {
    return "Critical";
  }
  if (state === "low") {
    return "Low";
  }
  if (state === "mismatch") {
    return "Mismatch";
  }
  if (state === "offline") {
    return "Waiting";
  }
  return "Good";
}

export function getBatteryState(
  connected: boolean,
  limits: BatteryVoltageLimits,
  now: number,
  power: HwPower | null,
  stats: SensorStats,
): BatteryState {
  if (!isHwPowerFresh(connected, power, stats, now) || !power?.battery_voltage_valid) {
    return "offline";
  }

  if (power.v_battery <= limits.critical) {
    return "critical";
  }
  if (power.v_battery <= limits.empty) {
    return "low";
  }
  if (power.drive_voltage_mismatch) {
    return "mismatch";
  }
  return "good";
}

export function getBatteryPercent(power: HwPower | null, fresh: boolean): number | null {
  if (!fresh || !power?.battery_voltage_valid || !Number.isFinite(power.battery_percentage)) {
    return null;
  }
  return clamp(power.battery_percentage, 0, 1);
}

function voltagePercent(limits: BatteryVoltageLimits, value: number | null): number {
  if (value === null || !Number.isFinite(value)) {
    return 0;
  }
  if (limits.full <= limits.critical) {
    return 0;
  }
  return clamp(((value - limits.critical) / (limits.full - limits.critical)) * 100, 0, 100);
}

function emptyMarkerPercent(limits: BatteryVoltageLimits): number {
  if (limits.full <= limits.critical) {
    return 0;
  }
  return clamp(((limits.empty - limits.critical) / (limits.full - limits.critical)) * 100, 0, 100);
}

function getVoltageState(
  connected: boolean,
  fresh: boolean,
  limits: BatteryVoltageLimits,
  power: HwPower | null,
  source: VoltageSource,
): BatteryState {
  const valid = Boolean(power?.[source.validKey]);
  const value = valid ? Number(power?.[source.voltageKey]) : Number.NaN;
  if (!connected || !fresh || !valid || !Number.isFinite(value)) {
    return "offline";
  }
  if (value <= limits.critical) {
    return "critical";
  }
  if (value <= limits.empty) {
    return "low";
  }
  if (source.usesDriveMismatch && power?.drive_voltage_mismatch) {
    return "mismatch";
  }
  return "good";
}

function voltageFreshnessLabel(connected: boolean, fresh: boolean, valid: boolean, stats: SensorStats, now: number): string {
  if (!connected) {
    return "Offline";
  }
  if (!fresh) {
    return stats.lastMessageAt ? `Stale ${formatAge(stats.lastMessageAt, now)}` : "No data";
  }
  return valid ? formatAge(stats.lastMessageAt, now) : "Invalid";
}

function VoltageCard({
  connected,
  fresh,
  limits,
  now,
  power,
  source,
  stats,
}: {
  connected: boolean;
  fresh: boolean;
  limits: BatteryVoltageLimits;
  now: number;
  power: HwPower | null;
  source: VoltageSource;
  stats: SensorStats;
}) {
  const valid = Boolean(power?.[source.validKey]);
  const state = getVoltageState(connected, fresh, limits, power, source);
  const rawValue = valid ? Number(power?.[source.voltageKey]) : Number.NaN;
  const value = fresh && valid && Number.isFinite(rawValue) ? rawValue : null;
  const style = {
    "--battery-empty-marker": `${emptyMarkerPercent(limits)}%`,
    "--battery-fill": `${voltagePercent(limits, value)}%`,
  } as CSSProperties;

  return (
    <article className={`battery-card is-${state}`}>
      <div className="battery-card-top">
        <div>
          <span>{source.label}</span>
          <strong>{source.detail}</strong>
        </div>
        <span className={`battery-badge is-${state}`}>{batteryStateLabel(state, connected)}</span>
      </div>

      <div className="battery-card-value">
        <strong>{formatVoltage(value)}</strong>
        <span>V</span>
      </div>

      <div className="battery-track" style={style} aria-hidden="true">
        <span />
        <i />
      </div>

      <div className="battery-card-foot">
        <span>{limits.empty.toFixed(1)} - {limits.full.toFixed(1)} V</span>
        <span>{voltageFreshnessLabel(connected, fresh, valid, stats, now)}</span>
      </div>
    </article>
  );
}

export function BatteryVoltagePanel({ connected, limits, now, power, stats }: BatteryVoltagePanelProps) {
  const fresh = isHwPowerFresh(connected, power, stats, now);
  const state = getBatteryState(connected, limits, now, power, stats);
  const percent = getBatteryPercent(power, fresh);
  const validVoltageCount = fresh
    ? VOLTAGE_SOURCES.filter((source) => Boolean(power?.[source.validKey])).length
    : 0;
  const warning = fresh ? (power?.warning ?? "").trim() : "";

  return (
    <section className="battery-panel" aria-label="Battery voltage output">
      <div className="scan-stage-header">
        <div>
          <span className="eyebrow">Battery</span>
          <h2>VESC input voltage</h2>
        </div>
        <div className={`battery-summary is-${state}`}>
          {state === "critical" || state === "low" || state === "mismatch" ? (
            <AlertTriangle size={18} aria-hidden="true" />
          ) : (
            <Battery size={18} aria-hidden="true" />
          )}
          <span>{batteryStateLabel(state, connected)}</span>
          <strong>{formatBatteryPercent(percent)}</strong>
        </div>
      </div>

      <div className="battery-grid">
        {VOLTAGE_SOURCES.map((source) => (
          <VoltageCard
            connected={connected}
            fresh={fresh}
            key={source.voltageKey}
            limits={limits}
            now={now}
            power={power}
            source={source}
            stats={stats}
          />
        ))}
      </div>

      <div className="battery-meta-row">
        <div>
          <span>Aggregate</span>
          <strong>{fresh && power?.battery_voltage_valid ? `${formatVoltage(power.v_battery)} V` : "--"}</strong>
        </div>
        <div>
          <span>Source</span>
          <strong>{fresh && power?.battery_source ? power.battery_source.replace(/_/g, " ") : "--"}</strong>
        </div>
        <div>
          <span>Valid inputs</span>
          <strong>{validVoltageCount}/{VOLTAGE_SOURCES.length}</strong>
        </div>
        <div>
          <span>Mismatch warn</span>
          <strong>{limits.mismatchWarn.toFixed(1)} V</strong>
        </div>
      </div>

      {(state === "critical" || state === "low" || state === "mismatch" || warning) && (
        <div className={`battery-alert is-${state}`}>
          <Zap size={16} aria-hidden="true" />
          <span>{warning || `${batteryStateLabel(state, connected)} battery voltage`}</span>
        </div>
      )}
    </section>
  );
}
