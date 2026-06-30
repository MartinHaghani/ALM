import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { RobotState, SensorStats } from "./types";

interface UseRobotStateOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseRobotStateResult {
  robotState: RobotState | null;
  stats: SensorStats;
}

export function useRobotState({ ros, topicName }: UseRobotStateOptions): UseRobotStateResult {
  const [robotState, setRobotState] = useState<RobotState | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setRobotState(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<RobotState>({
      messageType: "xbot_msgs/RobotState",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: 100,
    });

    topic.subscribe((message: RobotState) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setRobotState(message);
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  return { robotState, stats };
}
