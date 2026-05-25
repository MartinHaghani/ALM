import type { CSSProperties } from "react";

import {
  accelerationToTilt,
  hasValidQuaternion,
  isImuSampleValid,
  quaternionToEuler,
  STANDARD_GRAVITY,
  toDegrees,
  vectorMagnitude,
} from "./imuMath";
import type { Imu, ImuStats } from "./types";

interface ImuPanelProps {
  imu: Imu | null;
  now: number;
  stats: ImuStats;
}

const IMU_VISUAL_MAX_HZ = 30;

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
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

function formatDegrees(value: number | null): string {
  if (value === null || !Number.isFinite(value)) {
    return "--";
  }
  return `${formatNumber(value, 1)} deg`;
}

export function ImuPanel({ imu, now, stats }: ImuPanelProps) {
  const hasSample = isImuSampleValid(imu);
  const hasQuaternion = hasValidQuaternion(imu);
  const displayedHz = Math.min(stats.hz, IMU_VISUAL_MAX_HZ);
  const tilt = hasSample && imu ? accelerationToTilt(imu.linear_acceleration) : null;
  const euler = hasQuaternion && imu ? quaternionToEuler(imu.orientation) : null;

  const rollDeg = euler ? toDegrees(euler.roll) : tilt ? toDegrees(tilt.roll) : null;
  const pitchDeg = euler ? toDegrees(euler.pitch) : tilt ? toDegrees(tilt.pitch) : null;
  const yawDeg = euler ? toDegrees(euler.yaw) : null;

  const acceleration = imu?.linear_acceleration ?? null;
  const accelerationMagnitude = acceleration ? vectorMagnitude(acceleration) : Number.NaN;
  const accelerationG = accelerationMagnitude / STANDARD_GRAVITY;
  const xArrowLength = acceleration ? clamp(Math.abs(acceleration.x) / (STANDARD_GRAVITY * 1.5), 0, 1) * 72 : 0;
  const yArrowLength = acceleration ? clamp(Math.abs(acceleration.y) / (STANDARD_GRAVITY * 1.5), 0, 1) * 72 : 0;
  const zArrowLength = acceleration ? clamp(Math.abs(acceleration.z) / (STANDARD_GRAVITY * 1.5), 0, 1) * 72 : 0;
  const xArrowAngle = acceleration && acceleration.x < 0 ? 152 : -28;
  const yArrowAngle = acceleration && acceleration.y < 0 ? 62 : -118;
  const zArrowAngle = acceleration && acceleration.z < 0 ? 90 : -90;

  const mowerTransform =
    rollDeg === null || pitchDeg === null
      ? undefined
      : `rotateX(${58 + clamp(-pitchDeg, -18, 18)}deg) rotateY(-38deg) rotateZ(${-28 + clamp(rollDeg, -22, 22)}deg)`;
  const mowerStyle: CSSProperties = mowerTransform ? { transform: mowerTransform } : {};
  const xArrowStyle = { "--arrow-angle": `${xArrowAngle}deg`, "--arrow-length": `${xArrowLength}%` } as CSSProperties;
  const yArrowStyle = { "--arrow-angle": `${yArrowAngle}deg`, "--arrow-length": `${yArrowLength}%` } as CSSProperties;
  const zArrowStyle = { "--arrow-angle": `${zArrowAngle}deg`, "--arrow-length": `${zArrowLength}%` } as CSSProperties;

  return (
    <section className="imu-panel" aria-label="IMU sensor output">
      <div className="scan-stage-header">
        <div>
          <span className="eyebrow">IMU sensor</span>
          <h2>Orientation & acceleration</h2>
        </div>
        <div className="scan-state">
          <span>{formatNumber(displayedHz, 1)} Hz</span>
        </div>
      </div>

      <div className="imu-visual-block imu-visual-block-3d">
        <div className="imu-block-header">
          <span>3D mower</span>
          <strong>{hasQuaternion ? "Quaternion" : "Fixed heading + accel tilt"}</strong>
        </div>

        <div className={`imu-3d-stage ${hasSample ? "" : "is-empty"}`}>
          {hasSample ? (
            <>
              <span className="imu-ground-grid" />
              <span className="mower-shadow mower-shadow-3d" />
              <span className="accel-arrow accel-arrow-x" style={xArrowStyle}>
                <span>X</span>
              </span>
              <span className="accel-arrow accel-arrow-y" style={yArrowStyle}>
                <span>Y</span>
              </span>
              <span className="accel-arrow accel-arrow-z" style={zArrowStyle}>
                <span>Z</span>
              </span>
              <div className="mower-body mower-body-3d" style={mowerStyle} aria-hidden="true">
                <span className="mower-front" />
                <span className="mower-deck-line" />
                <span className="mower-wheel mower-wheel-front-left" />
                <span className="mower-wheel mower-wheel-front-right" />
                <span className="mower-wheel mower-wheel-rear-left" />
                <span className="mower-wheel mower-wheel-rear-right" />
              </div>
            </>
          ) : (
            <span>No IMU data</span>
          )}
        </div>

        <div className="imu-vector-legend" aria-label="Acceleration vector legend">
          <span className="legend-x">X accel</span>
          <span className="legend-y">Y accel</span>
          <span className="legend-z">Z accel</span>
          <strong>{formatNumber(accelerationG, 2)} g</strong>
        </div>

        <div className="imu-reading-row imu-reading-row-combined">
          <div>
            <span>Roll</span>
            <strong>{formatDegrees(rollDeg)}</strong>
          </div>
          <div>
            <span>Pitch</span>
            <strong>{formatDegrees(pitchDeg)}</strong>
          </div>
          <div>
            <span>Yaw</span>
            <strong>{formatDegrees(yawDeg)}</strong>
          </div>
          <div>
            <span>Ax</span>
            <strong>{formatNumber(acceleration?.x ?? Number.NaN, 2)}</strong>
          </div>
          <div>
            <span>Ay</span>
            <strong>{formatNumber(acceleration?.y ?? Number.NaN, 2)}</strong>
          </div>
          <div>
            <span>Az</span>
            <strong>{formatNumber(acceleration?.z ?? Number.NaN, 2)}</strong>
          </div>
          <div>
            <span>Age</span>
            <strong>{formatAge(stats.lastMessageAt, now)}</strong>
          </div>
        </div>
      </div>
    </section>
  );
}
