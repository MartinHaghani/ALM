import type { CSSProperties } from "react";
import { Activity, AlertTriangle, Thermometer } from "lucide-react";

import type { TemperatureReading, TemperatureReadings, TemperatureSensorDefinition } from "./useTemperatureSensors";

export type TemperatureState = "good" | "watch" | "hot" | "critical" | "offline";

export interface TemperatureSensorSpec extends TemperatureSensorDefinition {
  criticalHigh: number;
  detail: string;
  hotHigh: number;
  label: string;
  max: number;
  min: number;
  operatingLabel: string;
  watchHigh: number;
  criticalLow?: number;
  watchLow?: number;
}

interface TemperaturePanelProps {
  connected: boolean;
  now: number;
  readings: TemperatureReadings;
}

export const TEMPERATURE_STALE_MS = 6000;

export const TEMPERATURE_SENSORS: TemperatureSensorSpec[] = [
  {
    id: "om_left_esc_temp",
    label: "Drive L ESC",
    detail: "75100 V2 Pro",
    min: 0,
    max: 105,
    watchHigh: 70,
    hotHigh: 85,
    criticalHigh: 100,
    operatingLabel: "85 / 100 C",
  },
  {
    id: "om_right_esc_temp",
    label: "Drive R ESC",
    detail: "75100 V2 Pro",
    min: 0,
    max: 105,
    watchHigh: 70,
    hotHigh: 85,
    criticalHigh: 100,
    operatingLabel: "85 / 100 C",
  },
  {
    id: "om_mow_esc_temp",
    label: "Blade ESC",
    detail: "75100 V2 Pro",
    min: 0,
    max: 100,
    watchHigh: 70,
    hotHigh: 85,
    criticalHigh: 90,
    operatingLabel: "85 / 90 C",
  },
  {
    id: "om_pi_cpu_temp",
    label: "Pi CPU",
    detail: "Thermal zone",
    min: 0,
    max: 95,
    watchHigh: 70,
    hotHigh: 80,
    criticalHigh: 85,
    operatingLabel: "80 / 85 C",
  },
  {
    id: "om_imu_temp",
    label: "IMU",
    detail: "LSM6DSO",
    min: -40,
    max: 90,
    watchHigh: 70,
    hotHigh: 80,
    criticalHigh: 85,
    criticalLow: -40,
    watchLow: -20,
    operatingLabel: "-40 to 85 C",
  },
  {
    id: "om_gnss_temp",
    label: "GNSS",
    detail: "u-blox F9P",
    min: -40,
    max: 90,
    watchHigh: 70,
    hotHigh: 80,
    criticalHigh: 85,
    criticalLow: -40,
    watchLow: -20,
    operatingLabel: "-40 to 85 C",
  },
];

const STATE_WEIGHT: Record<TemperatureState, number> = {
  offline: 0,
  good: 1,
  watch: 2,
  hot: 3,
  critical: 4,
};

const STATE_LABELS: Record<TemperatureState, string> = {
  critical: "Critical",
  good: "Good",
  hot: "Hot",
  offline: "Waiting",
  watch: "Watch",
};

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

export function formatTemperature(value: number | null): string {
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

export function temperatureStateLabel(state: TemperatureState, connected: boolean): string {
  if (state === "offline" && !connected) {
    return "Offline";
  }
  return STATE_LABELS[state];
}

function freshnessLabel(connected: boolean, reading: TemperatureReading | undefined, fresh: boolean, now: number): string {
  if (!connected) {
    return "Offline";
  }
  if (fresh) {
    return formatAge(reading?.stats.lastMessageAt ?? null, now);
  }
  if (reading?.stats.lastMessageAt) {
    return `Stale ${formatAge(reading.stats.lastMessageAt, now)}`;
  }
  return "No data";
}

export function isReadingFresh(connected: boolean, reading: TemperatureReading | undefined, now: number): boolean {
  return Boolean(connected && reading?.data && reading.stats.lastMessageAt && now - reading.stats.lastMessageAt <= TEMPERATURE_STALE_MS);
}

export function getTemperatureState(
  spec: TemperatureSensorSpec,
  reading: TemperatureReading | undefined,
  connected: boolean,
  now: number,
): TemperatureState {
  if (!isReadingFresh(connected, reading, now)) {
    return "offline";
  }

  const value = reading?.data?.data;
  if (value === undefined || !Number.isFinite(value)) {
    return "offline";
  }

  if ((spec.criticalLow !== undefined && value <= spec.criticalLow) || value >= spec.criticalHigh) {
    return "critical";
  }
  if (value >= spec.hotHigh) {
    return "hot";
  }
  if ((spec.watchLow !== undefined && value <= spec.watchLow) || value >= spec.watchHigh) {
    return "watch";
  }
  return "good";
}

function temperaturePercent(spec: TemperatureSensorSpec, value: number | null): number {
  if (value === null || !Number.isFinite(value)) {
    return 0;
  }
  return clamp(((value - spec.min) / (spec.max - spec.min)) * 100, 0, 100);
}

function markerPercent(spec: TemperatureSensorSpec): number {
  return clamp(((spec.hotHigh - spec.min) / (spec.max - spec.min)) * 100, 0, 100);
}

export function getOverallState(states: TemperatureState[], connected: boolean): TemperatureState {
  if (!connected) {
    return "offline";
  }

  const liveStates = states.filter((state) => state !== "offline");
  if (liveStates.length === 0) {
    return "offline";
  }

  return liveStates.reduce((worst, state) => (STATE_WEIGHT[state] > STATE_WEIGHT[worst] ? state : worst), "good" as TemperatureState);
}

function TemperatureCard({
  connected,
  now,
  reading,
  spec,
}: {
  connected: boolean;
  now: number;
  reading: TemperatureReading | undefined;
  spec: TemperatureSensorSpec;
}) {
  const state = getTemperatureState(spec, reading, connected, now);
  const fresh = isReadingFresh(connected, reading, now);
  const value = fresh && reading?.data ? reading.data.data : null;
  const fillPercent = temperaturePercent(spec, value);
  const style = {
    "--temperature-fill": `${fillPercent}%`,
    "--temperature-marker": `${markerPercent(spec)}%`,
  } as CSSProperties;

  return (
    <article className={`temperature-card is-${state}`}>
      <div className="temperature-card-top">
        <div>
          <span>{spec.label}</span>
          <strong>{spec.detail}</strong>
        </div>
        <span className={`temperature-badge is-${state}`}>{temperatureStateLabel(state, connected)}</span>
      </div>

      <div className="temperature-card-value">
        <strong>{formatTemperature(value)}</strong>
        <span>&deg;C</span>
      </div>

      <div className="temperature-track" style={style} aria-hidden="true">
        <span />
        <i />
      </div>

      <div className="temperature-card-foot">
        <span>{spec.operatingLabel}</span>
        <span>{freshnessLabel(connected, reading, fresh, now)}</span>
      </div>
    </article>
  );
}

export function TemperaturePanel({ connected, now, readings }: TemperaturePanelProps) {
  const states = TEMPERATURE_SENSORS.map((spec) => getTemperatureState(spec, readings[spec.id], connected, now));
  const overallState = getOverallState(states, connected);
  const liveCount = states.filter((state) => state !== "offline").length;
  const alertCount = states.filter((state) => state === "hot" || state === "critical").length;

  return (
    <section className="temperature-panel" aria-label="Temperature sensor output">
      <div className="scan-stage-header">
        <div>
          <span className="eyebrow">Temperatures</span>
          <h2>Thermal health</h2>
        </div>
        <div className={`temperature-summary is-${overallState}`}>
          {overallState === "critical" || overallState === "hot" ? (
            <AlertTriangle size={18} aria-hidden="true" />
          ) : overallState === "offline" ? (
            <Activity size={18} aria-hidden="true" />
          ) : (
            <Thermometer size={18} aria-hidden="true" />
          )}
          <span>{temperatureStateLabel(overallState, connected)}</span>
          <strong>{liveCount}/{TEMPERATURE_SENSORS.length}</strong>
        </div>
      </div>

      <div className="temperature-grid">
        {TEMPERATURE_SENSORS.map((spec) => (
          <TemperatureCard
            connected={connected}
            key={spec.id}
            now={now}
            reading={readings[spec.id]}
            spec={spec}
          />
        ))}
      </div>

      {alertCount > 0 && (
        <div className={`temperature-alert is-${overallState}`}>
          <AlertTriangle size={16} aria-hidden="true" />
          <span>{alertCount} thermal value{alertCount === 1 ? "" : "s"} above the normal band</span>
        </div>
      )}
    </section>
  );
}
