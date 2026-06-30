import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { NavPath, SensorStats } from "./types";

interface UseNavPathOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseNavPathResult {
  path: NavPath | null;
  stats: SensorStats;
}

export function useNavPath({ ros, topicName }: UseNavPathOptions): UseNavPathResult {
  const [path, setPath] = useState<NavPath | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setPath(null);
    setStats(emptyStats());

    if (!ros || !topicName) {
      return undefined;
    }

    const topic = new Topic<NavPath>({
      messageType: "nav_msgs/Path",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: 100,
    });

    topic.subscribe((message: NavPath) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setPath(message);
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return { path, stats };
}
