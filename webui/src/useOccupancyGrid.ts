import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { MapStats, OccupancyGrid } from "./types";

interface UseOccupancyGridOptions {
  paused: boolean;
  resetKey?: number;
  ros: Ros | null;
  throttleMs?: number;
  topicName: string;
}

interface UseOccupancyGridResult {
  grid: OccupancyGrid | null;
  stats: MapStats;
}

export function useOccupancyGrid({
  paused,
  resetKey = 0,
  ros,
  throttleMs = 1000,
  topicName,
}: UseOccupancyGridOptions): UseOccupancyGridResult {
  const [grid, setGrid] = useState<OccupancyGrid | null>(null);
  const [stats, setStats] = useState<MapStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);
  const pausedRef = useRef(paused);

  useEffect(() => {
    pausedRef.current = paused;
  }, [paused]);

  useEffect(() => {
    arrivalsRef.current = [];
    setGrid(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<OccupancyGrid>({
      messageType: "nav_msgs/OccupancyGrid",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: throttleMs,
    });

    topic.subscribe((message: OccupancyGrid) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));

      if (!pausedRef.current) {
        setGrid(message);
      }
    });

    return () => {
      topic.unsubscribe();
    };
  }, [resetKey, ros, throttleMs, topicName]);

  return {
    grid,
    stats,
  };
}
