import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { MowerMapData, SensorStats, StringMessage } from "./types";

interface UseMowerMapOptions {
  ros: Ros | null;
  throttleMs?: number;
  topicName: string;
}

interface UseMowerMapResult {
  mapData: MowerMapData | null;
  stats: SensorStats;
}

function parseMowerMap(message: StringMessage): MowerMapData | null {
  try {
    const parsed = JSON.parse(message.data) as Partial<MowerMapData>;
    if (!Array.isArray(parsed.areas) || !Array.isArray(parsed.docking_stations)) {
      return null;
    }
    return {
      areas: parsed.areas,
      docking_stations: parsed.docking_stations,
    };
  } catch {
    return null;
  }
}

export function useMowerMap({ ros, throttleMs = 1000, topicName }: UseMowerMapOptions): UseMowerMapResult {
  const [mapData, setMapData] = useState<MowerMapData | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setMapData(null);
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
      setMapData(parseMowerMap(message));
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, throttleMs, topicName]);

  return { mapData, stats };
}
