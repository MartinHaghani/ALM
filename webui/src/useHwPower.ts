import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { HwPower, SensorStats } from "./types";

interface UseHwPowerOptions {
  ros: Ros | null;
  throttleMs?: number;
  topicName: string;
}

interface UseHwPowerResult {
  power: HwPower | null;
  stats: SensorStats;
}

export function useHwPower({ ros, throttleMs = 250, topicName }: UseHwPowerOptions): UseHwPowerResult {
  const [power, setPower] = useState<HwPower | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setPower(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<HwPower>({
      messageType: "mower_msgs/HwPower",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: throttleMs,
    });

    topic.subscribe((message: HwPower) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setPower(message);
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, throttleMs, topicName]);

  return { power, stats };
}
