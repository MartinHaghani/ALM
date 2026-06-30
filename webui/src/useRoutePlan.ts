import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { parseLiveRoutePlanPayload } from "./routePlan";
import { emptyStats, estimateHz } from "./rosStats";
import type { RoutePlan, SensorStats, StringMessage } from "./types";

interface UseRoutePlanOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseRoutePlanResult {
  routePlan: RoutePlan | null;
  stats: SensorStats;
}

export function useRoutePlan({ ros, topicName }: UseRoutePlanOptions): UseRoutePlanResult {
  const [routePlan, setRoutePlan] = useState<RoutePlan | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setRoutePlan(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<StringMessage>({
      messageType: "std_msgs/String",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: 100,
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
      setRoutePlan(parseLiveRoutePlanPayload(message.data));
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return { routePlan, stats };
}
