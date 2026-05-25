import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { Imu, ImuStats } from "./types";

interface UseImuOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseImuResult {
  imu: Imu | null;
  stats: ImuStats;
}

const IMU_UI_UPDATE_MS = 1000 / 30;

export function useImu({ ros, topicName }: UseImuOptions): UseImuResult {
  const [imu, setImu] = useState<Imu | null>(null);
  const [stats, setStats] = useState<ImuStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);
  const latestImuRef = useRef<Imu | null>(null);
  const latestStatsRef = useRef<ImuStats>(emptyStats());
  const pendingUpdateRef = useRef(false);

  useEffect(() => {
    arrivalsRef.current = [];
    latestImuRef.current = null;
    latestStatsRef.current = emptyStats();
    pendingUpdateRef.current = false;
    setImu(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<Imu>({
      messageType: "sensor_msgs/Imu",
      name: topicName,
      ros,
      throttle_rate: Math.round(IMU_UI_UPDATE_MS),
    });

    const flushTimer = window.setInterval(() => {
      if (!pendingUpdateRef.current) {
        return;
      }
      pendingUpdateRef.current = false;
      setImu(latestImuRef.current);
      setStats(latestStatsRef.current);
    }, IMU_UI_UPDATE_MS);

    topic.subscribe((message: Imu) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      latestStatsRef.current = {
        hz,
        lastMessageAt: now,
        messageCount: latestStatsRef.current.messageCount + 1,
      };
      latestImuRef.current = message;
      pendingUpdateRef.current = true;
    });

    return () => {
      window.clearInterval(flushTimer);
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return {
    imu,
    stats,
  };
}
