import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import type { LaserScan, ScanStats } from "./types";
import { emptyStats, estimateHz } from "./rosStats";

interface UseLaserScanOptions {
  paused: boolean;
  ros: Ros | null;
  topicName: string;
}

interface UseLaserScanResult {
  scan: LaserScan | null;
  stats: ScanStats;
}

export function useLaserScan({ paused, ros, topicName }: UseLaserScanOptions): UseLaserScanResult {
  const [scan, setScan] = useState<LaserScan | null>(null);
  const [stats, setStats] = useState<ScanStats>(emptyStats);

  const arrivalsRef = useRef<number[]>([]);
  const pausedRef = useRef(paused);

  useEffect(() => {
    pausedRef.current = paused;
  }, [paused]);

  useEffect(() => {
    arrivalsRef.current = [];
    setScan(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<LaserScan>({
      messageType: "sensor_msgs/LaserScan",
      name: topicName,
      ros,
      throttle_rate: 0,
    });

    topic.subscribe((message: LaserScan) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));

      if (!pausedRef.current) {
        setScan(message);
      }
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return {
    scan,
    stats,
  };
}
