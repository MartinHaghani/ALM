import type { SensorStats } from "./types";

export function emptyStats(): SensorStats {
  return {
    hz: 0,
    lastMessageAt: null,
    messageCount: 0,
  };
}

export function estimateHz(arrivals: number[]): number {
  if (arrivals.length < 2) {
    return 0;
  }

  const spanSeconds = (arrivals[arrivals.length - 1] - arrivals[0]) / 1000;
  if (spanSeconds <= 0) {
    return 0;
  }

  return (arrivals.length - 1) / spanSeconds;
}
