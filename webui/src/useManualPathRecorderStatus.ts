import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { ManualPathRecorderStatus, SensorStats, StringMessage } from "./types";

interface UseManualPathRecorderStatusOptions {
  ros: Ros | null;
  throttleMs?: number;
  topicName: string;
}

interface UseManualPathRecorderStatusResult {
  stats: SensorStats;
  status: ManualPathRecorderStatus | null;
}

function parseStatus(message: StringMessage): ManualPathRecorderStatus | null {
  try {
    return JSON.parse(message.data) as ManualPathRecorderStatus;
  } catch {
    return null;
  }
}

export function useManualPathRecorderStatus({
  ros,
  throttleMs = 500,
  topicName,
}: UseManualPathRecorderStatusOptions): UseManualPathRecorderStatusResult {
  const [status, setStatus] = useState<ManualPathRecorderStatus | null>(null);
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
      setStats((current) => ({
        hz: estimateHz(arrivalsRef.current),
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
