import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { MapOverlay, SensorStats } from "./types";

interface UseMapOverlayOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseMapOverlayResult {
  overlay: MapOverlay | null;
  stats: SensorStats;
}

export function useMapOverlay({ ros, topicName }: UseMapOverlayOptions): UseMapOverlayResult {
  const [overlay, setOverlay] = useState<MapOverlay | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setOverlay(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<MapOverlay>({
      messageType: "xbot_msgs/MapOverlay",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: 100,
    });

    topic.subscribe((message: MapOverlay) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setOverlay(message);
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return { overlay, stats };
}
