import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { AbsolutePose, GpsStatusStats } from "./types";

interface UseGpsStatusOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseGpsStatusResult {
  stats: GpsStatusStats;
  status: AbsolutePose | null;
}

export function useGpsStatus({ ros, topicName }: UseGpsStatusOptions): UseGpsStatusResult {
  const [status, setStatus] = useState<AbsolutePose | null>(null);
  const [stats, setStats] = useState<GpsStatusStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setStatus(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<AbsolutePose>({
      messageType: "xbot_msgs/AbsolutePose",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: 250,
    });

    topic.subscribe((message: AbsolutePose) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setStatus(message);
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return {
    stats,
    status,
  };
}
