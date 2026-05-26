import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { SensorStats, SlamAlignmentStatus, StringMessage } from "./types";

interface UseSlamAlignmentStatusOptions {
  ros: Ros | null;
  throttleMs?: number;
  topicName: string;
}

interface UseSlamAlignmentStatusResult {
  stats: SensorStats;
  status: SlamAlignmentStatus | null;
}

function parseStatus(message: StringMessage): SlamAlignmentStatus | null {
  try {
    return JSON.parse(message.data) as SlamAlignmentStatus;
  } catch {
    return null;
  }
}

export function useSlamAlignmentStatus({
  ros,
  throttleMs = 250,
  topicName,
}: UseSlamAlignmentStatusOptions): UseSlamAlignmentStatusResult {
  const [status, setStatus] = useState<SlamAlignmentStatus | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setStatus(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<StringMessage>({
      messageType: "std_msgs/String",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: throttleMs,
    });

    topic.subscribe((message: StringMessage) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setStatus(parseStatus(message));
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, throttleMs, topicName]);

  return { stats, status };
}
