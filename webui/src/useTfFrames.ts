import { useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { TfMessage, TfStats } from "./types";
import { toTransform2D, transformKey, type TransformMap } from "./tfMath";

interface UseTfFramesOptions {
  ros: Ros | null;
  tfStaticTopic: string;
  tfTopic: string;
  throttleMs?: number;
}

interface UseTfFramesResult {
  stats: TfStats;
  transforms: TransformMap;
}

export function useTfFrames({
  ros,
  tfStaticTopic,
  tfTopic,
  throttleMs = 20,
}: UseTfFramesOptions): UseTfFramesResult {
  const [transforms, setTransforms] = useState<TransformMap>(() => new Map());
  const [stats, setStats] = useState<TfStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setTransforms(new Map());
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const handleMessage = (message: TfMessage, isStatic = false) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);

      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));

      setTransforms((current) => {
        const next = new Map(current);
        for (const transform of message.transforms) {
          const transform2d = toTransform2D(transform, now, isStatic);
          next.set(transformKey(transform2d.parentFrame, transform2d.childFrame), transform2d);
        }
        return next;
      });
    };

    const tf = new Topic<TfMessage>({
      messageType: "tf2_msgs/TFMessage",
      name: tfTopic,
      queue_length: 1,
      ros,
      throttle_rate: throttleMs,
    });

    const tfStatic = new Topic<TfMessage>({
      messageType: "tf2_msgs/TFMessage",
      name: tfStaticTopic,
      queue_length: 1,
      ros,
      throttle_rate: 0,
    });

    tf.subscribe((message) => handleMessage(message, false));
    tfStatic.subscribe((message) => handleMessage(message, true));

    return () => {
      tf.unsubscribe();
      tfStatic.unsubscribe();
    };
  }, [ros, tfStaticTopic, tfTopic, throttleMs]);

  return {
    stats,
    transforms,
  };
}
