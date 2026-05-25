import type { Imu, Quaternion, Vector3 } from "./types";

export const STANDARD_GRAVITY = 9.80665;

export interface EulerAngles {
  pitch: number;
  roll: number;
  yaw: number;
}

export interface TiltAngles {
  pitch: number;
  roll: number;
}

export function toDegrees(radians: number): number {
  return (radians * 180) / Math.PI;
}

export function vectorMagnitude(vector: Vector3): number {
  return Math.hypot(vector.x, vector.y, vector.z);
}

export function isFiniteVector(vector: Vector3): boolean {
  return Number.isFinite(vector.x) && Number.isFinite(vector.y) && Number.isFinite(vector.z);
}

export function isImuSampleValid(imu: Imu | null): boolean {
  if (!imu) {
    return false;
  }

  return isFiniteVector(imu.linear_acceleration) && isFiniteVector(imu.angular_velocity);
}

export function hasValidQuaternion(imu: Imu | null): boolean {
  if (!imu || imu.orientation_covariance[0] === -1) {
    return false;
  }

  const { x, y, z, w } = imu.orientation;
  const norm = Math.hypot(x, y, z, w);
  return Number.isFinite(norm) && norm > 0.001;
}

export function quaternionToEuler(quaternion: Quaternion): EulerAngles {
  const norm = Math.hypot(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  const x = quaternion.x / norm;
  const y = quaternion.y / norm;
  const z = quaternion.z / norm;
  const w = quaternion.w / norm;

  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (w * y - z * x);
  const pitch = Math.asin(Math.max(-1, Math.min(1, sinp)));

  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  return {
    pitch,
    roll,
    yaw,
  };
}

export function accelerationToTilt(acceleration: Vector3): TiltAngles {
  const roll = Math.atan2(acceleration.y, acceleration.z);
  const pitch = Math.atan2(-acceleration.x, Math.hypot(acceleration.y, acceleration.z));

  return {
    pitch,
    roll,
  };
}
