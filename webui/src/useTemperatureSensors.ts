import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { SensorDataDouble, SensorStats } from "./types";

export interface TemperatureSensorDefinition {
  id: string;
  topicName?: string;
}

export interface TemperatureReading {
  data: SensorDataDouble | null;
  stats: SensorStats;
}

export type TemperatureReadings = Record<string, TemperatureReading>;

interface UseTemperatureSensorsOptions {
  ros: Ros | null;
  sensors: TemperatureSensorDefinition[];
  throttleMs?: number;
}

function createEmptyReadings(sensors: TemperatureSensorDefinition[]): TemperatureReadings {
  return Object.fromEntries(
    sensors.map((sensor) => [
      sensor.id,
      {
        data: null,
        stats: emptyStats(),
      },
    ]),
  );
}

export function useTemperatureSensors({
  ros,
  sensors,
  throttleMs = 500,
}: UseTemperatureSensorsOptions): TemperatureReadings {
  const [readings, setReadings] = useState<TemperatureReadings>(() => createEmptyReadings(sensors));
  const arrivalsRef = useRef<Record<string, number[]>>({});
  const sensorKey = sensors.map((sensor) => `${sensor.id}:${sensor.topicName ?? ""}`).join("|");

  useEffect(() => {
    arrivalsRef.current = {};
    setReadings(createEmptyReadings(sensors));

    if (!ros) {
      return undefined;
    }

    const topics = sensors.map((sensor) => {
      const topic = new Topic<SensorDataDouble>({
        messageType: "xbot_msgs/SensorDataDouble",
        name: sensor.topicName ?? `/xbot_monitoring/sensors/${sensor.id}/data`,
        queue_length: 1,
        ros,
        throttle_rate: throttleMs,
      });

      topic.subscribe((message: SensorDataDouble) => {
        const now = Date.now();
        const arrivals = [...(arrivalsRef.current[sensor.id] ?? []), now].filter((stamp) => now - stamp <= 5000);
        arrivalsRef.current[sensor.id] = arrivals;

        setReadings((current) => ({
          ...current,
          [sensor.id]: {
            data: message,
            stats: {
              hz: estimateHz(arrivals),
              lastMessageAt: now,
              messageCount: (current[sensor.id]?.stats.messageCount ?? 0) + 1,
            },
          },
        }));
      });

      return topic;
    });

    return () => {
      topics.forEach((topic) => topic.unsubscribe());
    };
  }, [ros, sensorKey, throttleMs]);

  return readings;
}
