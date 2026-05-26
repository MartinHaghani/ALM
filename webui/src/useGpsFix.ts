import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { GpsFixStats, NavSatFix } from "./types";

interface UseGpsFixOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseGpsFixResult {
  fix: NavSatFix | null;
  stats: GpsFixStats;
}

export function useGpsFix({ ros, topicName }: UseGpsFixOptions): UseGpsFixResult {
  const [fix, setFix] = useState<NavSatFix | null>(null);
  const [stats, setStats] = useState<GpsFixStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setFix(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<NavSatFix>({
      messageType: "sensor_msgs/NavSatFix",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: 250,
    });

    topic.subscribe((message: NavSatFix) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setFix(message);
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return {
    fix,
    stats,
  };
}
